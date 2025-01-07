from acados_template import AcadosModel
from casadi import SX, vertcat, cross
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def export_quadruped_ode_model() -> AcadosModel:
    logger.info("Creating quadruped ODE model...")
    
    # Create and verify model instance
    model = AcadosModel()
    if model is None:
        raise ValueError("Failed to create AcadosModel instance")
    
    # Set name first
    model.name = 'quadruped_ode'
    logger.debug(f"Created model with name: {model.name}")

    # constants
    m = 1    # mass of the robot (kg)
    I = .1    # inertia of the robot (kg*m^2)
    g = -9.81  # gravity (m/s^2)
    pc = SX.sym('pc',3,1) # position of the center of mass
    p1 = SX.sym('p1',3,1) # position of foot 1
    p2 = SX.sym('p2',3,1) # position of foot 2
    p3 = SX.sym('p3',3,1) # position of foot 3
    p4 = SX.sym('p4',3,1) # position of foot 4

    # Create state vector first
    states = [SX.sym(name) for name in ['x1', 'y1', 'z1', 'theta', 'phi', 'psi', 
                                       'vx', 'vy', 'vz', 'wx', 'wy', 'wz']]
    x = vertcat(*states)
    logger.debug(f"Created state vector with shape: {x.shape}")
    
    # Set state in model immediately
    model.x = x
    logger.debug(f"Set model.x with type: {type(model.x)}")
    
    # Set control variables
    F1 = SX.sym('F1', 3)
    F2 = SX.sym('F2', 3)
    F3 = SX.sym('F3', 3)
    F4 = SX.sym('F4', 3)
    u = vertcat(F1, F2, F3, F4)
    logger.debug(f"Control vector created with shape: {u.shape}")
    model.u = u
    
    # Store dimensions for cost computation
    model.nx = x.shape[0]
    model.nu = u.shape[0]

    # Create and set xdot immediately after state
    xdot = SX.sym('xdot', x.shape[0])
    model.xdot = xdot
    logger.debug(f"Set model.xdot with shape: {model.xdot.shape}")

    # Add reference state as a model attribute for external cost
    x_ref = SX.sym('x_ref', x.shape[0])
    model.x_ref = x_ref

    # Set parameters with dimension verification
    p = vertcat(p1, p2, p3, p4, pc)  # 15-dimensional (3×5)
    param_dim = p.shape[0]
    logger.debug(f"Parameter vector created with shape: {p.shape}")
    if param_dim != 15:
        raise ValueError(f"Parameter dimension mismatch: got {param_dim}, expected 15")
    model.p = p

    # dynamics
    # State positions for easier access
    q_pos = x[:3]           # position states [x1, y1, z1]
    q_rot = x[3:6]         # rotation states [theta, phi, psi]
    v_lin = x[6:9]         # linear velocities [vx, vy, vz]
    v_ang = x[9:12]        # angular velocities [wx, wy, wz]

    # System dynamics
    # Position and angle derivatives are just the velocities
    dq_pos = v_lin
    dq_rot = v_ang
    
    # Force dynamics (linear acceleration)
    dv_lin = vertcat(
        (F1[0] + F2[0] + F3[0] + F4[0])/m,
        (F1[1] + F2[1] + F3[1] + F4[1])/m,
        (F1[2] + F2[2] + F3[2] + F4[2])/m + g
    )
    
    # Torque dynamics (angular acceleration)
    dv_ang = (cross(p1 - pc, F1) + cross(p2 - pc, F2) + 
              cross(p3 - pc, F3) + cross(p4 - pc, F4)) / I

    # Combine all dynamics into state derivative vector
    f_expl = vertcat(dq_pos, dq_rot, dv_lin, dv_ang)
    
    # Set model dynamics
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl

    # Set labels
    model.x_labels = ['$x$ [m]', '$y$ [m]', '$z$ [m]', '$\\theta$ [rad]', '$\\phi$ [rad]', '$\\psi$ [rad]']
    model.u_labels = ['$F_1$ [N]', '$F_2$ [N]', '$F_3$ [N]', '$F_4$ [N]']
    model.t_label = '$t$ [s]'

    # Verify model before return
    if model.x is None:
        raise ValueError("model.x is None after setup")
    logger.info("Model creation completed successfully")
    
    return model