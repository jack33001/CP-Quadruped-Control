import numpy
import yaml
from acados_template import AcadosOcp, AcadosOcpSolver
from quadruped_model import export_quadruped_ode_model
import casadi as ca
import logging
import os

logger = logging.getLogger(__name__)

class QuadrupedOptimalController:
    def __init__(self, N=20, T=0.2, code_export_dir=None, param_file=None):
        logger.info(f"Initializing controller with N={N}, T={T}")
        self.N = N
        self.T = T

        # Load parameters from yaml
        if param_file is None:
            # Default to package config directory
            script_dir = os.path.dirname(os.path.abspath(__file__))
            root_dir = os.path.dirname(os.path.dirname(script_dir))
            param_file = os.path.join(root_dir, 'config', 'optimal_controller.yaml')
        
        with open(param_file, 'r') as f:
            params = yaml.safe_load(f)['optimal_controller']
            self.mass = params.get('mass', 1.0)
            self.inertia = params.get('inertia', 0.1)
            
        logger.info(f"Loaded parameters: mass={self.mass}kg, inertia={self.inertia}kg*m^2")

        # Get the quadruped_mpc root directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        root_dir = os.path.dirname(os.path.dirname(script_dir))
        
        # Default to acados_generated in include directory if not specified
        if code_export_dir is None:
            code_export_dir = os.path.join(root_dir, 'include', 'quadruped_mpc', 'acados_generated')
        
        logger.info(f"Will export code to: {code_export_dir}")
        os.makedirs(code_export_dir, exist_ok=True)
        os.makedirs(os.path.join(code_export_dir, 'quadruped_ode_model'), exist_ok=True)
        
        # Create and verify model with loaded parameters
        self.model = export_quadruped_ode_model(mass=self.mass, inertia=self.inertia)
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

        # Selection matrices (track position, orientation, and velocities)
        ocp.cost.Vx = numpy.eye(nx)   # Track all states
        ocp.cost.Vu = numpy.zeros((nx, nu))  # No direct control cost
        ocp.cost.Vx_e = ocp.cost.Vx.copy()  # Terminal cost same structure

        # Balanced weights for all state components
        pos_weights = [10.0, 10.0, 20.0]    # Back to original weights
        rot_weights = [10.0]*3              # Original rotation weights
        vel_weights = [1.0, 1.0, 2.0]       # Original velocity weights
        ang_weights = [1.0]*3               # Original angular weights
        
        # Combine all weights
        ocp.cost.W = numpy.diag(pos_weights + rot_weights + vel_weights + ang_weights)
        ocp.cost.W_e = ocp.cost.W * 10.0  # Stronger terminal cost

        # Reference vectors (will be updated in solve())
        ocp.cost.yref = numpy.zeros(nx)    # Full state reference
        ocp.cost.yref_e = numpy.zeros(nx)  # Terminal reference

        min_force = numpy.array([-self.mass, -self.mass, 0.0] * 4)     # Reduced lateral forces
        max_force = numpy.array([self.mass, self.mass, self.mass*25] * 4)      # Max vertical ~2.5x robot weight
        ocp.constraints.idxbu = numpy.arange(nu)
        ocp.constraints.lbu = min_force
        ocp.constraints.ubu = max_force

        # Remove ALL state constraints - let the controller work with actual state

        # No ocp.constraints.x0, idxbx_0, lbx_0, ubx_0, idxbx, lbx, or ubx

        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'  # Use HPIPM instead
        ocp.solver_options.hpipm_mode = 'BALANCE'  # Set HPIPM to balance mode
        ocp.solver_options.qp_solver_cond_N = self.N  # Full condensing
        ocp.solver_options.qp_solver_warm_start = 2  # More aggressive warm start
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.qp_solver_cond_N = self.N  # Number of stages to condense
        ocp.solver_options.qp_solver_cond_ric_alg = 0  # Use classical Riccati for condensing
        ocp.solver_options.qp_solver_ric_alg = 0       # Use classical Riccati for QP
        ocp.solver_options.nlp_solver_max_iter = 200
        ocp.solver_options.qp_solver_iter_max = 100
        
        # set prediction horizon
        ocp.solver_options.tf = self.T

        ocp.dims.nbx_0 = nx
        ocp.constraints.idxbx_0 = numpy.arange(nx)
        ocp.constraints.lbx_0 = numpy.zeros(nx)  # Will be updated in solve()
        ocp.constraints.ubx_0 = numpy.zeros(nx)  # Will be updated in solve()

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

    def solve(self, x0, x_ref, p1, p2, p3, p4, com):
        try:            
            # Set current state
            self.solver.set(0, 'x', x0)
            
            # Set target reference for future stages only (1 to N)
            # Don't set any reference for stage 0 since that's our current state
            for i in range(self.N + 1):
                self.solver.set(i, 'yref', x_ref)
            
            # Update foot positions and COM
            self.update_foot_positions(p1, p2, p3, p4, com)
            
            # Add parameter check before solving
            if hasattr(self.solver, 'acados_ocp'):
                param_size = self.solver.acados_ocp.dims.np
                if param_size != 15:
                    print(f"\nERROR: Incorrect parameter dimension: {param_size}, expected 15")
                    raise ValueError(f"Incorrect parameter dimension: {param_size}, expected 15")
            
            # Re-set the initial constraints - set entire vectors at once
            self.solver.constraints_set(0, 'lbx', x0)  # Set full vector
            self.solver.constraints_set(0, 'ubx', x0)  # Set full vector
            
            # Solve and check intermediate results if possible
            status = self.solver.solve()
            
            # get solution
            u0 = self.solver.get(0, 'u')
            
            return u0, status
            
        except Exception as e:
            print(f"\nSOLVER ERROR: {str(e)}")
            raise