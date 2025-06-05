from acados_template import AcadosModel
from casadi import SX, vertcat, cross
import logging
import yaml
import os

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def export_quadruped_ode_model(mass=None, inertia=None, config_file=None) -> AcadosModel:
    logger.info(" =========================== Creating quadruped ODE model =========================== ")
    
    # Try to get parameters from config file if not provided directly
    if mass is None or inertia is None:
        try:
            if config_file is None:
                # Default to consolidated config file
                script_dir = os.path.dirname(os.path.abspath(__file__))
                root_dir = os.path.dirname(os.path.dirname(script_dir))
                config_file = os.path.join(root_dir, 'config', 'quadruped_controllers.yaml')
            
            if os.path.exists(config_file):
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                    balance_params = config.get('balance_controller', {}).get('ros__parameters', {})
                    
                    if mass is None:
                        mass = balance_params.get('mass', 13.2)
                    if inertia is None:
                        inertia = balance_params.get('inertia', 0.5)
                        
                logger.info(f"Got parameters from config file: mass={mass}, inertia={inertia}")
            else:
                # Use defaults if file not found
                if mass is None:
                    mass = 13.2
                if inertia is None:
                    inertia = 0.5
                logger.info(f"Config file not found. Using default parameters: mass={mass}, inertia={inertia}")
        except Exception as e:
            logger.warning(f"Could not get parameters from config file: {e}")
            # Use defaults if there's an exception
            if mass is None:
                mass = 13.2
            if inertia is None:
                inertia = 0.5
            logger.info(f"Using default parameters: mass={mass}, inertia={inertia}")
    
    # Create and verify model instance
    model = AcadosModel()
    if model is None:
        raise ValueError("Failed to create AcadosModel instance")
    
    # Set name first
    model.name = 'quadruped_ode'
    logger.info(f"Created model with name: {model.name}")

    # constants
    m = mass      # mass of the robot (kg)
    I = inertia   # inertia of the robot (kg*m^2)
    g = -9.81  # gravity (m/s^2)

    # Create reduced state vector (13 states instead of 25)
    states = [SX.sym(name) for name in [# positions/orientations
                                        'x', 'y', 'z', 'q_w', 'q_x', 'q_y', 'q_z', 
                                        # velocities
                                        'vx', 'vy', 'vz', 'wx', 'wy', 'wz']]
    x = vertcat(*states)
    logger.info(f"Created state vector with shape: {x.shape}")
    logger.info(f"State vector: {x}")
    
    # Set state in model immediately
    model.x = x
    logger.info(f"Set model.x with type: {type(model.x)}")
    
    # Create parameters for foot positions (12 parameters: 4 feet * 3 coordinates)
    foot_params = [SX.sym(name) for name in ['f1_x', 'f1_y', 'f1_z',
                                             'f2_x', 'f2_y', 'f2_z',
                                             'f3_x', 'f3_y', 'f3_z',
                                             'f4_x', 'f4_y', 'f4_z']]
    p = vertcat(*foot_params)
    model.p = p
    logger.info(f"Created parameter vector with shape: {p.shape}")
    
    # Set control variables
    F1 = SX.sym('F1', 3)
    F2 = SX.sym('F2', 3)
    F3 = SX.sym('F3', 3)
    F4 = SX.sym('F4', 3)
    u = vertcat(F1, F2, F3, F4)
    logger.info(f"Control vector created with shape: {u.shape}")
    model.u = u
    
    # Store dimensions for cost computation
    model.nx = x.shape[0]
    model.nu = u.shape[0]
    model.np = p.shape[0]

    # Create and set xdot immediately after state
    xdot = SX.sym('xdot', x.shape[0])
    model.xdot = xdot
    logger.info(f"Set model.xdot with shape: {model.xdot.shape}")

    # Add reference state as a model attribute for external cost
    x_ref = SX.sym('x_ref', x.shape[0])
    model.x_ref = x_ref

    # dynamics
    # State positions for easier access
    q_pos = x[:3]           # position states [x1, y1, z1]
    q_quat = x[3:7]        # quaternion states [qw, qx, qy, qz]
    v_lin = x[7:10]        # linear velocities [vx, vy, vz]
    v_ang = x[10:13]       # angular velocities [wx, wy, wz]
    
    # Extract foot positions from parameters
    p1 = p[0:3]            # position of foot 1 [x,y,z]
    p2 = p[3:6]            # position of foot 2 [x,y,z]
    p3 = p[6:9]            # position of foot 3 [x,y,z]
    p4 = p[9:12]           # position of foot 4 [x,y,z]
    pc = q_pos

    # System dynamics
    # Position and angle derivatives are just the velocities
    dq_pos = v_lin
    logger.info(f"dq_pos: {dq_pos}")
    dq_rot = v_ang
    # Convert angular velocity to quaternion derivative
    dq_rot = 0.5 * vertcat(
        -q_quat[1]*v_ang[0] - q_quat[2]*v_ang[1] - q_quat[3]*v_ang[2],  # dq_w
        q_quat[0]*v_ang[0] + q_quat[2]*v_ang[2] - q_quat[3]*v_ang[1],   # dq_x
        q_quat[0]*v_ang[1] - q_quat[1]*v_ang[2] + q_quat[3]*v_ang[0],   # dq_y
        q_quat[0]*v_ang[2] + q_quat[1]*v_ang[1] - q_quat[2]*v_ang[0]    # dq_z
    )
    logger.info(f"dq_pos: {dq_rot}")
    
    # Force dynamics (linear acceleration)
    dv_lin = vertcat(
        (F1[0] + F2[0] + F3[0] + F4[0])/m,  # Revert back to original
        (F1[1] + F2[1] + F3[1] + F4[1])/m,
        (F1[2] + F2[2] + F3[2] + F4[2])/m + g  # g is already negative, so add it
    )
    logger.info(f"dv_lin: {dv_lin}")
    
    # Torque dynamics (angular acceleration)
    dv_ang = (cross(p1 - pc, F1) + cross(p2 - pc, F2) + 
              cross(p3 - pc, F3) + cross(p4 - pc, F4)) / I
    logger.info(f"dv_ang: {dv_ang}")

    # Combine all dynamics into state derivative vector (13 states only)
    f_expl = vertcat(dq_pos, dq_rot, dv_lin, dv_ang)
    logger.info(f"Created dynamics vector with shape: {f_expl.shape}")
    
    # Set model dynamics
    model.f_impl_expr = xdot - f_expl  
    model.f_expl_expr = f_expl 
    logger.info("Set model dynamics expressions")
    
    # Properly set initial conditions constraint
    model.x0 = x
    logger.info("Set initial conditions constraint")
    
    # Add dynamics constraints
    model.disc_dyn_expr = f_expl      
    model.dyn_type = 'explicit'      
    logger.info("Added dynamics constraints with type: explicit")
    
    # Set model dimensions explicitly
    model.nx = x.shape[0]
    model.nu = u.shape[0]
    model.np = p.shape[0]
    logger.info(f"Set model dimensions - nx: {model.nx}, nu: {model.nu}, np: {model.np}")

    # Set labels
    model.x_labels = ['$x$ [m]', '$y$ [m]', '$z$ [m]', '$\\theta$ [rad]', '$\\phi$ [rad]', '$\\psi$ [rad]']
    model.u_labels = ['$F_1$ [N]', '$F_2$ [N]', '$F_3$ [N]', '$F_4$ [N]']
    model.t_label = '$t$ [s]'
    logger.info("Set model labels for states, controls, and time")

    # Verify model before return
    if model.x is None:
        raise ValueError("model.x is None after setup")
    logger.info("Model creation completed successfully")
    
    return model