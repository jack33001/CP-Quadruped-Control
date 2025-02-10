from acados_template import AcadosModel
from casadi import SX, vertcat, cross
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def export_quadruped_ode_model(mass, inertia) -> AcadosModel:
    logger.info("Creating quadruped ODE model...")
    
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

    # Create state vector first
    states = [SX.sym(name) for name in [# positions
                                        'x', 'y', 'z', 'theta', 'phi', 'psi', 
                                        # velocities
                                        'vx', 'vy', 'vz', 'wx', 'wy', 'wz',
                                        # foot positions
                                        'f1_x', 'f1_y', 'f1_z',
                                        'f2_x', 'f2_y', 'f2_z',
                                        'f3_x', 'f3_y', 'f3_z',
                                        'f4_x', 'f4_y', 'f4_z']]
    x = vertcat(*states)
    logger.info(f"Created state vector with shape: {x.shape}")
    logger.info(f"State vector: {x}")
    
    # Set state in model immediately
    model.x = x
    logger.info(f"Set model.x with type: {type(model.x)}")
    
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
    q_rot = x[3:6]         # rotation states [theta, phi, psi]
    v_lin = x[6:9]         # linear velocities [vx, vy, vz]
    v_ang = x[9:12]        # angular velocities [wx, wy, wz]
    p1 = x[12:15]      # position of foot 1 [x,y,z]
    p2 = x[15:18]      # position of foot 2 [x,y,z]
    p3 = x[18:21]      # position of foot 3 [x,y,z]
    p4 = x[21:24]      # position of foot 4 [x,y,z]
    pc = q_pos

    # System dynamics
    # Position and angle derivatives are just the velocities
    dq_pos = v_lin
    logger.info(f"dq_pos: {dq_pos}")
    dq_rot = v_ang
    logger.info(f"dq_pos: {dq_pos}")
    
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

    # Combine all dynamics into state derivative vector
    # Add zero derivatives for foot positions (they are treated as parameters)
    foot_derivatives = vertcat(SX.zeros(3), SX.zeros(3), SX.zeros(3), SX.zeros(3))  # 12 zeros for 4 feet
    f_expl = vertcat(dq_pos, dq_rot, dv_lin, dv_ang, foot_derivatives)
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
    model.np = 0
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