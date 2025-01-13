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

        # Force bounds - adjusted for 1kg robot
        min_force = numpy.array([-5.0, -5.0, 0.0] * 4)     # Reduced lateral forces
        max_force = numpy.array([5.0, 5.0, 10.0] * 4)      # Max vertical ~2.5x robot weight
        ocp.constraints.idxbu = numpy.arange(nu)
        ocp.constraints.lbu = min_force
        ocp.constraints.ubu = max_force

        # Tighter state bounds for better stabilization
        ocp.constraints.idxbx = numpy.array([2])    # z-position index
        ocp.constraints.lbx = numpy.array([0.14])   # Tighter minimum height
        ocp.constraints.ubx = numpy.array([0.16])   # Tighter maximum height

        # Initial state constraint setup - allow specified z height
        ocp.constraints.x0 = numpy.zeros(nx)
        ocp.constraints.idxbx_0 = numpy.arange(nx)
        ocp.constraints.lbx_0 = numpy.zeros(nx)
        ocp.constraints.ubx_0 = numpy.zeros(nx)
        
        # Allow z-position (index 2) to match our desired initial height
        ocp.constraints.lbx_0[2] = 0.15  # Match initial z height
        ocp.constraints.ubx_0[2] = 0.15  # Match initial z height

        # set options
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'  # Changed solver
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # Changed to RTI
        ocp.solver_options.nlp_solver_max_iter = 200
        ocp.solver_options.qp_solver_iter_max = 100
        
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

    def solve(self, x0, x_ref, p1, p2, p3, p4, com):
        try:            
            # Change initial state setting
            self.solver.set(0, 'x', x0)
            
            # set reference trajectory
            yref = x_ref
            
            # set references for all nodes including terminal
            for i in range(self.N):
                self.solver.set(i, 'yref', yref)
            self.solver.set(self.N, 'yref', yref)
            
            # Update foot positions and COM
            self.update_foot_positions(p1, p2, p3, p4, com)
            
            # Add parameter check before solving
            if hasattr(self.solver, 'acados_ocp'):
                param_size = self.solver.acados_ocp.dims.np
                if param_size != 15:
                    print(f"\nERROR: Incorrect parameter dimension: {param_size}, expected 15")
                    raise ValueError(f"Incorrect parameter dimension: {param_size}, expected 15")
            
            # solve OCP
            print("Before solve:")
            print(f"Initial state: {x0}")
            print(f"Initial state in solver config: {self.solver.get(0, 'x')}")
            print(f"Difference: {x0 - self.solver.get(0, 'x')}")
            status = self.solver.solve()
            print("\nAfter solve:")
            print(f"Initial state: {x0}")
            print(f"Initial state in solver config: {self.solver.get(0, 'x')}")
            print(f"Difference: {x0 - self.solver.get(0, 'x')}")
            
            # get solution
            u0 = self.solver.get(0, 'u')
            
            # Print formatted tables
            print("\n=== New Control Step ===")
            
            # State and Reference Table
            print("\nState vs Reference:")
            print("┌──────────┬────────────┬────────────┬────────────┐")
            print("│ Variable │   Current  │  Reference │    Error   │")
            print("├──────────┼────────────┼────────────┼────────────┤")
            state_names = ['pos_x', 'pos_y', 'pos_z', 
                         'roll', 'pitch', 'yaw',
                         'vel_x', 'vel_y', 'vel_z',
                         'ang_x', 'ang_y', 'ang_z']
            for i, name in enumerate(state_names):
                print(f"│{name:^10}│{x0[i]:^11.3f} │{x_ref[i]:^11.3f} │{x_ref[i]-x0[i]:^11.3f} │")
            print("└──────────┴────────────┴────────────┴────────────┘")
            
            # Foot Forces Table
            print("\nComputed Foot Forces:")
            print("┌──────┬────────────┬────────────┬────────────┐")
            print("│ Foot │    F_x     │    F_y     │    F_z     │")
            print("├──────┼────────────┼────────────┼────────────┤")
            feet = ['FR', 'FL', 'BR', 'BL']
            for i, foot in enumerate(feet):
                fx = u0[i*3]
                fy = u0[i*3 + 1]
                fz = u0[i*3 + 2]
                print(f"│ {foot:^4} │{fx:^11.3f} │{fy:^11.3f} │{fz:^11.3f} │")
            print("└──────┴────────────┴────────────┴────────────┘")
            
            # Add solver stages table
            print("\nSolver Stages Analysis:")
            print("┌──────┬────────────┬────────────┬────────────┬────────────┬────────────┐")
            print("│ Stage│   COM_z    │  State z   │   Ref z    │ State v_z  │  Ref v_z   │")
            print("├──────┼────────────┼────────────┼────────────┼────────────┼────────────┤")
            for i in range(self.N + 1):
                com_z = com[2] if com is not None else 0.0
                state_z = self.solver.get(i, 'x')[2]
                ref_z = x_ref[2]
                state_vz = self.solver.get(i, 'x')[8]
                ref_vz = x_ref[8]
                print(f"│{i:^6}│{com_z:^11.3f} │{state_z:^11.3f} │{ref_z:^11.3f} │{state_vz:^11.3f} │{ref_vz:^11.3f} │")
            print("└──────┴────────────┴────────────┴────────────┴────────────┴────────────┘")
            
            print("\n----------------------------------------------------------------------------------------------------------------------\n")
            
            return u0, status
            
        except Exception as e:
            print(f"\nSOLVER ERROR: {str(e)}")
            raise