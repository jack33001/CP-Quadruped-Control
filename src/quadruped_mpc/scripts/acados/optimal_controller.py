import numpy
import yaml
from acados_template import AcadosOcp, AcadosOcpSolver
from quadruped_model import export_quadruped_ode_model
import casadi as ca
import logging
import os

logger = logging.getLogger(__name__)

class QuadrupedOptimalController:
    def __init__(self, N=None, T=None, code_export_dir=None, param_file=None):
        logger.info(f"Initializing controller with N={N}, T={T}")
        self.N = N
        self.T = T

        # Load parameters from yaml
        if param_file is None:
            # Default to consolidated config file
            script_dir = os.path.dirname(os.path.abspath(__file__))
            root_dir = os.path.dirname(os.path.dirname(script_dir))
            param_file = os.path.join(root_dir, 'config', 'quadruped_controllers.yaml')
        
        try:
            with open(param_file, 'r') as f:
                config = yaml.safe_load(f)
                balance_params = config.get('balance_controller', {}).get('ros__parameters', {})
                
                # Get parameters with defaults
                if N is None:
                    self.N = balance_params.get('stages', 50)
                if T is None:
                    self.T = balance_params.get('horizon', 5.0)
                
                # Robot parameters
                self.mass = balance_params.get('mass', 13.2)
                self.inertia = balance_params.get('inertia', 0.5)
                
                # Load weight parameters
                weights = balance_params.get('weights', {})
                self.position_weights = weights.get('position', [100000000, 100000000, 100000000])
                self.orientation_weights = weights.get('orientation', [1000000, 1000000, 1000000, 1000000])
                self.velocity_weights = weights.get('velocity', [1000, 1000, 1000])
                self.angular_velocity_weights = weights.get('angular_velocity', [10000, 10000, 10000])
                self.terminal_weight_factor = weights.get('terminal_factor', 10.0)
                
        except Exception as e:
            logger.warning(f"Error loading config file: {e}. Using default parameters.")
            # Fall back to default values if there's an error
            if self.N is None:
                self.N = 50
            if self.T is None:
                self.T = 5.0
                
            self.mass = 13.2
            self.inertia = 0.5
            self.position_weights = [100000000, 100000000, 100000000]
            self.orientation_weights = [1000000, 1000000, 1000000, 1000000]
            self.velocity_weights = [1000, 1000, 1000]
            self.angular_velocity_weights = [10000, 10000, 10000]
            self.terminal_weight_factor = 10.0
            
        logger.info(f"Loaded parameters: mass={self.mass}kg, inertia={self.inertia}kg*m^2")
        logger.info(f"Loaded weights: position={self.position_weights}, orientation={self.orientation_weights}, "
                   f"velocity={self.velocity_weights}, angular_velocity={self.angular_velocity_weights}, "
                   f"terminal_factor={self.terminal_weight_factor}")

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
        self.model = export_quadruped_ode_model(mass=self.mass, inertia=self.inertia, config_file=param_file)
        if self.model.x is None:
            raise ValueError("Model state is None after creation")
        logger.info(f"Created model with state type: {type(self.model.x)}")
        
        # Initialize OCP with model
        self.ocp = AcadosOcp()
        self.ocp.model = self.model  # Set model explicitly
        logger.info("Set model in OCP")
        
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
                    logger.info(f"Copied {item}")
                elif os.path.isdir(src) and item == 'quadruped_ode_model':
                    if os.path.exists(dst):
                        shutil.rmtree(dst)
                    shutil.copytree(src, dst)
                    logger.info(f"Copied dir {item}")
            
            # Copy ACADOS types.h to proper location
            types_src = os.path.join(self.ocp.acados_include_dir, 'acados', 'utils', 'types.h')
            types_dst = os.path.join(dest_dir, 'acados', 'utils', 'types.h')
            if os.path.exists(types_src):
                shutil.copy2(types_src, types_dst)
                logger.info(f"Copied types.h to {types_dst}")
            
            # Clean up
            shutil.rmtree(c_generated)
            logger.info(f"Generated code moved to: {dest_dir}")
        
    def setup_ocp(self):
        logger.info(" =========================== Setting up OCP  =========================== ")
        ocp = self.ocp
        model = self.model
        
        # Verify model attributes
        logger.info(f"Model state shape: {model.x.shape}")
        logger.info(f"Model control shape: {model.u.shape}")
        
        # Get dimensions from model
        nx = model.x.shape[0]  # 25 (state dimension)
        nu = model.u.shape[0]  # 12 (control dimension)
        np = 0
        ny = 13              # We only track 13 states in the cost
        ny_e = 13           # Terminal cost tracks same states

        logger.info(f"Dimensions - nx: {nx}, nu: {nu}, np: {np}, ny: {ny}, ny_e: {ny_e}")

        # Set dimensions
        ocp.dims.nx = nx
        ocp.dims.nu = nu
        ocp.dims.np = 0
        ocp.dims.ny = ny      # Track only 13 states
        ocp.dims.ny_e = ny_e  # Same for terminal cost
        ocp.dims.N = self.N

        # Selection matrices - only select first 13 states
        Vx = numpy.zeros((ny, nx))
        Vx[:13, :13] = numpy.eye(13)  # Select only first 13 states
        Vu = numpy.zeros((ny, nu))*.001 # for now, don't worry about policing the controller effort
        
        ocp.cost.Vx = Vx
        ocp.cost.Vu = Vu
        ocp.cost.Vx_e = Vx  # Terminal cost same selection
        logger.info("Set selection matrices to track only first 13 states")

        # Weights for tracked states - use values from YAML
        pos_weights = self.position_weights
        rot_weights = self.orientation_weights
        vel_weights = self.velocity_weights
        ang_weights = self.angular_velocity_weights
        
        # Combine weights into diagonal matrices
        W = numpy.diag(pos_weights + rot_weights + vel_weights + ang_weights)
        W_e = W * self.terminal_weight_factor  # Terminal cost higher to enforce convergence
        
        ocp.cost.W = W
        ocp.cost.W_e = W_e
        logger.info("Set weight matrices from configuration file")

        # References (only for tracked states)
        ocp.cost.yref = numpy.zeros(ny)     # Reference for tracked states
        ocp.cost.yref_e = numpy.zeros(ny_e) # Terminal reference
        logger.info("Set zero references")

        # Force constraints
        # Constrain friction to below mu = .2, constrain force to positive values 4x what's necessary to lift the robot
        min_force = numpy.array([-self.mass*9.81 / 4 * .2,-self.mass*9.81 / 4 * .2, 0] * 4)
        max_force = numpy.array([self.mass*9.81 / 4 * .2, self.mass*9.81 / 4 * .2, self.mass*9.81] * 4)
        ocp.constraints.idxbu = numpy.arange(nu)
        ocp.constraints.lbu = min_force
        ocp.constraints.ubu = max_force
        logger.info(f"Set force constraints: min={min_force[0]}, max={max_force[0]}")

        # Solver options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hpipm_mode = 'BALANCE'
        ocp.solver_options.qp_solver_cond_N = self.N
        ocp.solver_options.qp_solver_warm_start = 2
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.qp_solver_cond_ric_alg = 0
        ocp.solver_options.qp_solver_ric_alg = 0
        ocp.solver_options.nlp_solver_max_iter = 200
        ocp.solver_options.qp_solver_iter_max = 100
        logger.info("Set solver options")
        
        # Prediction horizon
        ocp.solver_options.tf = self.T
        logger.info(f"Set prediction horizon to {self.T}s")

        # Initial state bounds
        ocp.dims.nbx_0 = nx
        ocp.constraints.idxbx_0 = numpy.arange(nx)
        ocp.constraints.x0 = numpy.zeros(nx)
        logger.info("Set initial state constraints")

        logger.info("OCP setup completed")

    def solve(self, x0, x_ref):
        try:         
            self.solver.yref_0 = x0[:13]
            # Set the reference trajectory
            for i in range(self.N):
                self.solver.set(i, "yref", x_ref[:13])  # Only first 13 states as per cost setup
            self.solver.set(self.N, "yref", x_ref[:13])  # Terminal reference
            
            u0 = self.solver.solve_for_x0(x0)
            status = 0
            logger.debug(f"Got control solution with shape: {u0.shape}")
            
            return u0, status
            
        except Exception as e:
            logger.error(f"Solver error: {str(e)}")
            raise