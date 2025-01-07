#!/usr/bin/env python3

# Configure logging before imports
import logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s: %(message)s'
)
# Suppress matplotlib debug output
logging.getLogger('matplotlib').setLevel(logging.WARNING)

import numpy as np
from optimal_controller import QuadrupedOptimalController
import matplotlib.pyplot as plt
from time import time

logger = logging.getLogger(__name__)

def calculate_metrics(x_hist, u_hist, x_ref, sim_time):
    """Calculate performance metrics from simulation data"""
    # Add simulation timing
    steps_completed = len(x_hist) - 1  # -1 because x_hist includes initial state
    
    # Position RMS error
    pos_error = np.sqrt(np.mean((x_hist[:, :3] - x_ref[:3])**2, axis=0))
    
    # Orientation RMS error
    ori_error = np.sqrt(np.mean((x_hist[:, 3:6] - x_ref[3:6])**2, axis=0))
    
    # Force statistics
    max_forces = np.max(np.abs(u_hist), axis=0).reshape(4, 3)
    avg_forces = np.mean(np.abs(u_hist), axis=0).reshape(4, 3)
    
    # Stability metrics
    avg_height = np.mean(x_hist[:, 2])
    height_var = np.std(x_hist[:, 2])
    roll_var = np.std(x_hist[:, 3])
    pitch_var = np.std(x_hist[:, 4])
    
    metrics = {
        'sim_time': sim_time,
        'steps_completed': steps_completed,
        'avg_step_time': sim_time / steps_completed if steps_completed > 0 else 0,
        'pos_error': pos_error,
        'ori_error': ori_error,
        'max_forces': max_forces,
        'avg_forces': avg_forces,
        'avg_height': avg_height,
        'height_var': height_var,
        'roll_var': roll_var,
        'pitch_var': pitch_var
    }
    
    return metrics

def system_dynamics(x, u, m, I, p1, p2, p3, p4, com):
    """Calculate state derivatives given current state and control"""
    dx = np.zeros(12)
    dx[0:6] = x[6:12]         # velocities affect positions
    
    # Reshape control output into foot forces
    F1 = u[0:3]   # front right foot force
    F2 = u[3:6]   # front left foot force
    F3 = u[6:9]   # back right foot force
    F4 = u[9:12]  # back left foot force
    
    # Sum all foot forces for linear acceleration
    total_force = F1 + F2 + F3 + F4
    dx[6:9] = total_force/m   # linear accelerations
    dx[8] += -9.81            # add gravity to z acceleration
    
    # Calculate torques from foot forces
    r1 = p1 - com  # vectors from COM to each foot
    r2 = p2 - com
    r3 = p3 - com
    r4 = p4 - com
    
    # Sum up torques from each foot
    total_torque = (np.cross(r1, F1) + 
                   np.cross(r2, F2) + 
                   np.cross(r3, F3) + 
                   np.cross(r4, F4))
    dx[9:12] = total_torque/I  # angular accelerations
    
    return dx

def rk4_step(x, u, dt, m, I, p1, p2, p3, p4, com):
    """Perform one RK4 integration step"""
    k1 = system_dynamics(x, u, m, I, p1, p2, p3, p4, com)
    k2 = system_dynamics(x + dt/2 * k1, u, m, I, p1, p2, p3, p4, com)
    k3 = system_dynamics(x + dt/2 * k2, u, m, I, p1, p2, p3, p4, com)
    k4 = system_dynamics(x + dt * k3, u, m, I, p1, p2, p3, p4, com)
    
    return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

def main():
    # Robot physical parameters
    m = 1  # mass in kg
    I = .1  # inertia in kg*m^2
    
    # Initialize controller
    controller = QuadrupedOptimalController(N=20, T=0.2)
    
    # Set up simulation parameters
    dt = 0.01  # simulation timestep
    t_final = 2.0  # simulation duration
    n_steps = int(t_final / dt)
    
    # Initial state [x, y, z, theta, phi, psi, vx, vy, vz, wx, wy, wz]
    x0 = np.zeros(12)
    x0[2] = 0.3  # Start at higher z for safety
    
    # Target state - setpoint at standing position
    x_ref = np.zeros(12)
    x_ref[2] = 0.15  # Desired standing height
    
    # Example foot positions (in robot frame)
    leg_length = 0.15  # Robot leg length
    p1 = np.array([leg_length, leg_length, 0.0])   # front right
    p2 = np.array([leg_length, -leg_length, 0.0])  # front left
    p3 = np.array([-leg_length, leg_length, 0.0])  # back right
    p4 = np.array([-leg_length, -leg_length, 0.0]) # back left
    
    # Initialize COM position at target height
    com = np.array([0.0, 0.0, 0.15])
    
    # Update foot positions in the controller
    controller.update_foot_positions(p1, p2, p3, p4, com)
    
    # Initialize history arrays correctly for 12-dimensional state
    x_hist = np.zeros((n_steps+1, 12))  # Pre-allocate full state history
    u_hist = np.zeros((n_steps, 12))    # Pre-allocate control history
    t_hist = np.zeros(n_steps+1)        # Pre-allocate time history
    
    # Initial state setup
    x = x0.copy()
    x_hist[0] = x0  # Store initial state
    t_hist[0] = 0.0
    
    # Simple simulation loop
    success = False
    sim_start_time = time()
    try:
        for i in range(n_steps):
            t = i * dt
            
            # Get optimal control input
            try:
                u, status = controller.solve(x, x_ref)
                if status != 0:  # Non-zero status means solve failed
                    print(f"\nSOLVER ERROR: Failed at t={t:.3f} with status {status}")
                    break
                success = True  # Only mark success if status is 0
            except Exception as e:
                print(f"\nSOLVER ERROR: Exception at t={t:.3f}: {str(e)}")
                break
            
            # RK4 integration step
            x = rk4_step(x, u, dt, m, I, p1, p2, p3, p4, com)
            
            # Store data (pre-allocated arrays)
            t_hist[i+1] = t + dt
            x_hist[i+1] = x
            u_hist[i] = u
            
        # After simulation loop, before plotting
        if success:
            sim_time = time() - sim_start_time
            # Calculate and log metrics
            metrics = calculate_metrics(x_hist, u_hist, x_ref, sim_time)
            
            logger.info("\n=== Controller Performance Metrics ===")
            logger.info(f"Simulation time: {metrics['sim_time']:.3f} s")
            logger.info(f"Steps completed: {metrics['steps_completed']}")
            logger.info(f"Average step time: {metrics['avg_step_time']*1000:.2f} ms")
            
            logger.info(f"\nReference State:")
            logger.info(f"Position (x,y,z): [{', '.join(f'{x:.3f}' for x in x_ref[:3])}] m")
            logger.info(f"Orientation (r,p,y): [{', '.join(f'{x:.3f}' for x in x_ref[3:6])}] rad")
            logger.info(f"Linear vel (vx,vy,vz): [{', '.join(f'{x:.3f}' for x in x_ref[6:9])}] m/s")
            logger.info(f"Angular vel (wx,wy,wz): [{', '.join(f'{x:.3f}' for x in x_ref[9:12])}] rad/s")
            
            logger.info(f"\nFinal State:")
            logger.info(f"Position (x,y,z): [{', '.join(f'{x:.3f}' for x in x_hist[-1,:3])}] m")
            logger.info(f"Orientation (r,p,y): [{', '.join(f'{x:.3f}' for x in x_hist[-1,3:6])}] rad")
            logger.info(f"Linear vel (vx,vy,vz): [{', '.join(f'{x:.3f}' for x in x_hist[-1,6:9])}] m/s")
            logger.info(f"Angular vel (wx,wy,wz): [{', '.join(f'{x:.3f}' for x in x_hist[-1,9:12])}] rad/s")
            
            logger.info(f"\nTracking Performance:")
            logger.info(f"Position RMS Error (x,y,z): [{', '.join(f'{x:.3f}' for x in metrics['pos_error'])}] m")
            logger.info(f"Orientation RMS Error (r,p,y): [{', '.join(f'{x:.3f}' for x in metrics['ori_error'])}] rad")
            logger.info("\nForce Statistics (N):")
            for i, foot in enumerate(['FR', 'FL', 'BR', 'BL']):
                max_force = metrics['max_forces'][i]
                avg_force = metrics['avg_forces'][i]
                logger.info(f"{foot} max forces (x,y,z): [{', '.join(f'{x:.3f}' for x in max_force)}]")
                logger.info(f"{foot} avg forces (x,y,z): [{', '.join(f'{x:.3f}' for x in avg_force)}]")
            logger.info("\nStability Metrics:")
            logger.info(f"Average height: {metrics['avg_height']:.3f} m")
            logger.info(f"Height variance: {metrics['height_var']:.3f} m")
            logger.info(f"Roll variance: {metrics['roll_var']:.3f} rad")
            logger.info(f"Pitch variance: {metrics['pitch_var']:.3f} rad")
            
            # Continue with plotting
            plt.figure(figsize=(12, 8))
            
            plt.subplot(2, 1, 1)
            for i, label in enumerate(['x', 'y', 'z']):
                plt.plot(t_hist, x_hist[:, i], label=label)
            plt.grid(True)
            plt.legend()
            plt.title('Position vs Time')
            
            plt.subplot(2, 1, 2)
            for i, label in enumerate(['theta', 'phi', 'psi']):
                plt.plot(t_hist, x_hist[:, i+3], label=label)
            plt.grid(True)
            plt.legend()
            plt.title('Orientation vs Time')
            
            plt.tight_layout()
            plt.show()
        else:
            print("No successful iterations to plot")
            
    except Exception as e:
        print(f"Simulation failed: {str(e)}")
        if len(x_hist) > 1:  # If we have some data, try to plot it
            print("Attempting to plot partial data...")
            x_hist = np.array(x_hist)
            t_hist = np.array(t_hist)
            # ... plotting code ...

if __name__ == "__main__":
    main()