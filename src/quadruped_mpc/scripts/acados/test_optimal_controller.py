#!/usr/bin/env python3

# Configure logging before imports
import logging
from datetime import datetime
import os

# Create logs directory if it doesn't exist
log_dir = os.path.join(os.path.dirname(__file__), 'logs')
os.makedirs(log_dir, exist_ok=True)

# Configure file and console logging
log_file = os.path.join(log_dir, f'controller_test_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s: %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
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
    metrics = {}
    
    # Timing metrics
    metrics['sim_time'] = sim_time
    metrics['steps_completed'] = len(x_hist) - 1
    metrics['avg_step_time'] = sim_time / metrics['steps_completed'] if metrics['steps_completed'] > 0 else 0
    
    # Position metrics
    metrics['pos_error'] = np.sqrt(np.mean((x_hist[:, :3] - x_ref[:3])**2, axis=0))
    metrics['pos_max_error'] = np.max(np.abs(x_hist[:, :3] - x_ref[:3]), axis=0)
    metrics['pos_drift'] = x_hist[-1, :3] - x_hist[0, :3]
    
    # Orientation metrics
    metrics['ori_error'] = np.sqrt(np.mean((x_hist[:, 3:6] - x_ref[3:6])**2, axis=0))
    metrics['ori_max_error'] = np.max(np.abs(x_hist[:, 3:6] - x_ref[3:6]), axis=0)
    metrics['ori_drift'] = x_hist[-1, 3:6] - x_hist[0, 3:6]
    
    # Velocity metrics
    metrics['vel_mean'] = np.mean(np.abs(x_hist[:, 6:9]), axis=0)
    metrics['ang_vel_mean'] = np.mean(np.abs(x_hist[:, 9:12]), axis=0)
    
    # Force metrics
    metrics['max_forces'] = np.max(np.abs(u_hist), axis=0).reshape(4, 3)
    metrics['avg_forces'] = np.mean(np.abs(u_hist), axis=0).reshape(4, 3)
    metrics['force_std'] = np.std(u_hist, axis=0).reshape(4, 3)
    
    # Energy metrics
    metrics['total_force_magnitude'] = np.sum(np.linalg.norm(u_hist.reshape(-1, 4, 3), axis=2))
    metrics['avg_power'] = np.mean(np.sum(np.abs(u_hist.reshape(-1, 4, 3) * 
                                                x_hist[:-1, 6:9].reshape(-1, 1, 3)), axis=(1,2)))
    
    # Stability metrics
    metrics['avg_height'] = np.mean(x_hist[:, 2])
    metrics['height_var'] = np.std(x_hist[:, 2])
    metrics['roll_var'] = np.std(x_hist[:, 3])
    metrics['pitch_var'] = np.std(x_hist[:, 4])
    metrics['yaw_var'] = np.std(x_hist[:, 5])
    
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

def log_metrics(metrics, logger):
    """Log all metrics in a structured format"""
    logger.info("\n============ Controller Performance Analysis ============")
    
    # Timing Performance
    logger.info("\n=== Timing Performance ===")
    logger.info(f"Total simulation time: {metrics['sim_time']:.3f} s")
    logger.info(f"Steps completed: {metrics['steps_completed']}")
    logger.info(f"Average step time: {metrics['avg_step_time']*1000:.2f} ms")
    
    # Tracking Performance
    logger.info("\n=== Position Tracking ===")
    logger.info(f"RMS Error (x,y,z): [{', '.join(f'{x:.3f}' for x in metrics['pos_error'])}] m")
    logger.info(f"Max Error (x,y,z): [{', '.join(f'{x:.3f}' for x in metrics['pos_max_error'])}] m")
    logger.info(f"Position Drift: [{', '.join(f'{x:.3f}' for x in metrics['pos_drift'])}] m")
    
    logger.info("\n=== Orientation Tracking ===")
    logger.info(f"RMS Error (r,p,y): [{', '.join(f'{x:.3f}' for x in metrics['ori_error'])}] rad")
    logger.info(f"Max Error (r,p,y): [{', '.join(f'{x:.3f}' for x in metrics['ori_max_error'])}] rad")
    logger.info(f"Orientation Drift: [{', '.join(f'{x:.3f}' for x in metrics['ori_drift'])}] rad")
    
    # Velocity Performance
    logger.info("\n=== Velocity Performance ===")
    logger.info(f"Mean Linear Velocity (x,y,z): [{', '.join(f'{x:.3f}' for x in metrics['vel_mean'])}] m/s")
    logger.info(f"Mean Angular Velocity (x,y,z): [{', '.join(f'{x:.3f}' for x in metrics['ang_vel_mean'])}] rad/s")
    
    # Force Performance
    logger.info("\n=== Force Performance ===")
    for i, foot in enumerate(['FR', 'FL', 'BR', 'BL']):
        logger.info(f"\n{foot} Foot:")
        logger.info(f"Max forces (x,y,z): [{', '.join(f'{x:.3f}' for x in metrics['max_forces'][i])}] N")
        logger.info(f"Avg forces (x,y,z): [{', '.join(f'{x:.3f}' for x in metrics['avg_forces'][i])}] N")
        logger.info(f"Force std (x,y,z): [{', '.join(f'{x:.3f}' for x in metrics['force_std'][i])}] N")
    
    # Energy Performance
    logger.info("\n=== Energy Performance ===")
    logger.info(f"Total force magnitude: {metrics['total_force_magnitude']:.3f} N")
    logger.info(f"Average power: {metrics['avg_power']:.3f} W")
    
    # Stability Performance
    logger.info("\n=== Stability Metrics ===")
    logger.info(f"Average height: {metrics['avg_height']:.3f} m")
    logger.info(f"Height variance: {metrics['height_var']:.3f} m")
    logger.info(f"Roll variance: {metrics['roll_var']:.3f} rad")
    logger.info(f"Pitch variance: {metrics['pitch_var']:.3f} rad")
    logger.info(f"Yaw variance: {metrics['yaw_var']:.3f} rad")

def main():
    logger.info("Starting controller test...")
    # Robot physical parameters
    m = 1  # mass in kg
    I = .1  # inertia in kg*m^2
    
    # Initialize controller
    controller = QuadrupedOptimalController(N=20, T=0.2)
    
    # Set up simulation parameters
    dt = 0.01  # simulation timestep
    t_final = 10  # simulation duration
    n_steps = int(t_final / dt)
    
    # Initial state [x, y, z, theta, phi, psi, vx, vy, vz, wx, wy, wz]
    x0 = np.zeros(12)
    x0[2] = 0.15  # Changed from 0.3 to match height constraints (0.14-0.16)
    
    # Target state - setpoint at standing position
    x_ref = np.zeros(12)
    x_ref[2] = 0.15  # Target height matches initial height
    
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
            logger.debug(f"Step {i}/{n_steps} (t={t:.3f}s)")
            
            # Get optimal control input
            try:
                solve_start = time()
                u, status = controller.solve(x, x_ref)
                solve_time = time() - solve_start
                
                if status != 0:
                    logger.error(f"Solver failed at t={t:.3f} with status {status}")
                    break
                logger.debug(f"Solve time: {solve_time*1000:.2f}ms")
                success = True
            except Exception as e:
                logger.error(f"Solver exception at t={t:.3f}: {str(e)}")
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
            log_metrics(metrics, logger)
            
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
            
            # Save data
            data_file = os.path.join(log_dir, f'simulation_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.npz')
            np.savez(data_file, 
                    x_hist=x_hist, 
                    u_hist=u_hist, 
                    t_hist=t_hist,
                    x_ref=x_ref,
                    metrics=metrics)
            logger.info(f"Simulation data saved to {data_file}")
            
        else:
            logger.error("No successful iterations to plot")
            
    except Exception as e:
        logger.error(f"Simulation failed: {str(e)}", exc_info=True)
        
    # ...existing error handling code...

if __name__ == "__main__":
    main()