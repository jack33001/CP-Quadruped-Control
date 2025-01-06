import numpy
from acados_template import AcadosOcp, AcadosOcpSolver
from quadruped_model import export_quadruped_ode_model
import casadi as ca
import logging
import os

logger = logging.getLogger(__name__)

class QuadrupedOptimalController:
    def __init__(self, N=20, T=0.2, code_export_dir=None):
        logger.info(f"Initializing controller with N={N}, T={T}")
        self.N = N
        self.T = T

        # Get the quadruped_mpc root directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        root_dir = os.path.dirname(os.path.dirname(script_dir))
        
        # Default to acados_generated in include directory if not specified
        if code_export_dir is None:
            code_export_dir = os.path.join(root_dir, 'include', 'quadruped_mpc', 'acados_generated')
        
        logger.info(f"Will export code to: {code_export_dir}")
        os.makedirs(code_export_dir, exist_ok=True)
        os.makedirs(os.path.join(code_export_dir, 'quadruped_ode_model'), exist_ok=True)
        
        # Create and verify model
        self.model = export_quadruped_ode_model()
        if self.model.x is None:
            raise ValueError("Model state is None after creation")
        logger.debug(f"Created model with state type: {type(self.model.x)}")
        
        # Initialize OCP with model
        self.ocp = AcadosOcp()
        self.ocp.model = self.model  # Set model explicitly
        logger.debug("Set model in OCP")
        
        # Setup OCP
        self.setup_ocp()
        
        # Add code generation options
        self.code_export_dir = code_export_dir
        
        # Configure code generation options in the OCP
        self.ocp.code_export_directory = code_export_dir
        
        # Create solver - this will automatically generate C code
        logger.info("Creating solver...")
        self.solver = AcadosOcpSolver(self.ocp, 
                                    json_file="acados_ocp.json")
        
        # Code is generated during solver creation, just need to move files
        logger.info("Moving generated files...")
        self._copy_generated_code(code_export_dir)

    def _copy_generated_code(self, dest_dir):
        """Copy generated C code to the target directory"""
        import shutil
        import glob
        
        # Default ACADOS generated code location
        c_generated = "c_generated_code"
        if os.path.exists(c_generated):
            # Create destination directories
            os.makedirs(os.path.join(dest_dir, 'quadruped_ode_model'), exist_ok=True)
            os.makedirs(os.path.join(dest_dir, 'acados', 'utils'), exist_ok=True)
            
            # Copy all files
            for item in os.listdir(c_generated):
                src = os.path.join(c_generated, item)
                dst = os.path.join(dest_dir, item)
                if os.path.isfile(src):
                    shutil.copy2(src, dst)
                    logger.debug(f"Copied {item}")
                elif os.path.isdir(src) and item == 'quadruped_ode_model':
                    if os.path.exists(dst):
                        shutil.rmtree(dst)
                    shutil.copytree(src, dst)
                    logger.debug(f"Copied dir {item}")
            
            # Copy ACADOS types.h to proper location
            types_src = os.path.join(self.ocp.acados_include_dir, 'acados', 'utils', 'types.h')
            types_dst = os.path.join(dest_dir, 'acados', 'utils', 'types.h')
            if os.path.exists(types_src):
                shutil.copy2(types_src, types_dst)
                logger.debug(f"Copied types.h to {types_dst}")
            
            # Clean up
            shutil.rmtree(c_generated)
            logger.info(f"Generated code moved to: {dest_dir}")
        
    def setup_ocp(self):
        logger.info("Setting up OCP...")
        ocp = self.ocp
        model = self.model
        
        # Verify model attributes
        logger.debug(f"Model state shape: {model.x.shape}")
        logger.debug(f"Model control shape: {model.u.shape}")
        
        # Get dimensions from model
        nx = model.x.shape[0]  # Should be 12
        nu = model.u.shape[0]  # Should be 12
        np = model.p.shape[0]  # Should be 15 (3 coords × 5 points: 4 feet + COM)
        ny = nx               # Cost dimension should match state dimension
        ny_e = nx            # Terminal cost dimension
        
        logger.info(f"Dimensions - nx: {nx}, nu: {nu}, np: {np}, ny: {ny}, ny_e: {ny_e}")

        # Set dimensions
        ocp.dims.nx = nx
        ocp.dims.nu = nu
        ocp.dims.np = np  # Must be set before parameter_values
        ocp.dims.N = self.N

        # Initialize parameter values with correct size (must be after setting np)
        p_init = numpy.zeros(np)  # 15 values for 5 points (4 feet + COM) × 3 coordinates each
        ocp.parameter_values = p_init
        logger.debug(f"Initialized parameters with shape: {p_init.shape}")

        # Switch to LINEAR_LS cost type instead of EXTERNAL
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'

        # Define stage cost matrices
        nx = model.x.shape[0]
        nu = model.u.shape[0]

        # Selection matrices
        ocp.cost.Vx = numpy.eye(nx)   # State selection matrix
        ocp.cost.Vu = numpy.zeros((nx, nu))  # No direct control cost
        ocp.cost.Vx_e = numpy.eye(nx)  # Terminal state selection

        # Weight matrices
        pos_weights = [1.0]*3    # Position tracking
        rot_weights = [1.0]*3    # Orientation tracking
        vel_weights = [.1]*3     # Linear velocity
        ang_weights = [.1]*3     # Angular velocity
        ocp.cost.W = numpy.diag(pos_weights + rot_weights + vel_weights + ang_weights)
        ocp.cost.W_e = ocp.cost.W  # Same weights for terminal cost

        # Reference vectors (will be updated in solve())
        ocp.cost.yref = numpy.zeros(nx)
        ocp.cost.yref_e = numpy.zeros(nx)

        # Input bounds (keep existing constraints)
        ocp.constraints.idxbu = numpy.arange(nu)
        ocp.constraints.lbu = -100 * numpy.ones(nu)
        ocp.constraints.ubu = 100 * numpy.ones(nu)
        
        # Initial state constraint setup
        ocp.constraints.x0 = numpy.zeros(nx)
        ocp.constraints.idxbx_0 = numpy.arange(nx)
        ocp.constraints.lbx_0 = numpy.zeros(nx)
        ocp.constraints.ubx_0 = numpy.zeros(nx)

        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP'
        
        # set prediction horizon
        ocp.solver_options.tf = self.T

        logger.info("OCP setup completed")
        
    def update_foot_positions(self, p1, p2, p3, p4, com=None):
        """Update foot positions and COM in the model
        
        Args:
            p1, p2, p3, p4: np.array(3,) - foot positions
            com: np.array(3,) - center of mass position (optional)
        """
        if com is None:
            com = numpy.zeros(3)
        
        # Concatenate all parameters (15 total: 3 coords × 5 points)
        p = numpy.concatenate([p1, p2, p3, p4, com])
        logger.debug(f"Updating parameters with shape: {p.shape}")
        
        # Set parameter values in OCP
        self.ocp.parameter_values = p
        
        # Update parameters for all nodes in the solver
        try:
            for i in range(self.N + 1):
                self.solver.set(i, 'p', p)
        except Exception as e:
            logger.error(f"Failed to update parameters: {str(e)}")
            raise

    def solve(self, x0, x_ref):
        """Solve the optimal control problem"""
        try:
            # Change initial state setting
            self.solver.set(0, 'x', x0)  # Changed from lbx/ubx to just x
            
            # set reference trajectory (only state reference, no control reference needed)
            yref = x_ref  # Changed: removed concatenation since we only need state reference
            
            # set references for all nodes including terminal
            for i in range(self.N):
                self.solver.set(i, 'yref', yref)
            self.solver.set(self.N, 'yref', yref)
            
            # Add parameter check before solving
            if hasattr(self.solver, 'acados_ocp'):
                param_size = self.solver.acados_ocp.dims.np
                logger.debug(f"Solver parameter size: {param_size}")
                if param_size != 15:  # Expected: 3 coords × (4 feet + COM)
                    raise ValueError(f"Incorrect parameter dimension: {param_size}, expected 15")
            
            # solve OCP
            status = self.solver.solve()
            
            # get solution
            u0 = self.solver.get(0, 'u')
            return u0, status
        except Exception as e:
            logger.error(f"Error during solve: {str(e)}")
            raise