#!/usr/bin/env python3

# Configure logging before imports
import logging
import yaml
from datetime import datetime
import os
from tqdm.auto import tqdm

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

def system_dynamics(x, u, foot_positions, m, I):
    """Calculate state derivatives given current state and control"""
    dx = np.zeros(13)  # Reduced to 13 states
    
    # Linear velocities are easy
    dx[0:3] = x[7:10]
    
    # Angular velocities need to be converted to quaternion derivative
    dx[3:7] = .5 * np.array([
        -x[4]*x[10] - x[5]*x[11] - x[6]*x[12],
        x[3]*x[10] + x[5]*x[12] - x[6]*x[11],
        x[3]*x[11] - x[4]*x[12] + x[6]*x[10],
        x[3]*x[12] + x[4]*x[11] - x[5]*x[10]
        ])
    
    # Unpack foot positions from parameters
    p1 = foot_positions[0:3]
    p2 = foot_positions[3:6]
    p3 = foot_positions[6:9]
    p4 = foot_positions[9:12]
    com = x[:3]
    
    # Reshape control output into foot forces
    F1 = u[0:3]   # front right foot force
    F2 = u[3:6]   # front left foot force
    F3 = u[6:9]   # back right foot force
    F4 = u[9:12]  # back left foot force
    
    # Sum all foot forces for linear acceleration
    total_force = F1 + F2 + F3 + F4
    dx[7:10] = total_force/m   # linear accelerations
    dx[9] += -9.81            # add gravity to z acceleration
    
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
    dx[10:13] = total_torque/I  # angular accelerations
    
    return dx

def rk4_step(x, u, foot_positions, dt, m, I):
    """Perform one RK4 integration step"""
    k1 = system_dynamics(x, u, foot_positions, m, I)
    k2 = system_dynamics(x + dt/2 * k1, u, foot_positions, m, I)
    k3 = system_dynamics(x + dt/2 * k2, u, foot_positions, m, I)
    k4 = system_dynamics(x + dt * k3, u, foot_positions, m, I)
    
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

def normalize_quaternion(q):
    """Normalize a quaternion to unit length."""
    norm = np.sqrt(np.sum(q**2))
    if norm < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm

def main():
    logger.info("Starting controller test...")
    
    # Get config file path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(os.path.dirname(script_dir))
    config_file = os.path.join(root_dir, 'config', 'quadruped_controllers.yaml')
    
    # Load parameters from yaml
    with open(config_file, 'r') as f:
        params = yaml.safe_load(f)['balance_controller']['ros__parameters']
        m = params.get('mass')
        I = params.get('inertia')
        N = params.get('stages')
        T = params.get('horizon')
    
    logger.info(f"Loaded parameters: mass={m}kg, inertia={I}kg*m^2")
    
    # Initialize controller with config file
    controller = QuadrupedOptimalController(N, T, param_file=config_file)
    
    # Set up simulation parameters
    dt = 0.001  # simulation timestep
    t_final = 5  # simulation duration
    n_steps = int(t_final / dt)
    
    # Example foot positions (in robot frame)
    leg_length = 0.15  # Robot leg length
    # Add random perturbations to foot positions
    foot_variation = .03
    p1 = np.array([leg_length + np.random.uniform(-foot_variation, foot_variation), leg_length + np.random.uniform(-foot_variation, foot_variation), 0.0])   # front right
    p2 = np.array([leg_length + np.random.uniform(-foot_variation, foot_variation), -leg_length + np.random.uniform(-foot_variation, foot_variation), 0.0])  # front left
    p3 = np.array([-leg_length + np.random.uniform(-foot_variation, foot_variation), leg_length + np.random.uniform(-foot_variation, foot_variation), 0.0])  # back right
    p4 = np.array([-leg_length + np.random.uniform(-foot_variation, foot_variation), -leg_length + np.random.uniform(-foot_variation, foot_variation), 0.0]) # back left
    
    # Concatenate foot positions into parameter vector
    foot_positions = np.concatenate([p1, p2, p3, p4])
    
    # Print foot positions for verification
    logger.info("Foot positions:")
    logger.info(f"FR (p1): {p1}")
    logger.info(f"FL (p2): {p2}")
    logger.info(f"BR (p3): {p3}")
    logger.info(f"BL (p4): {p4}")
    
    # Create reduced 13-dimensional state vector
    # Initial state with random perturbations within reasonable bounds
    x0_full = np.zeros(13)  # Reduced state dimension
    # Position bounds (in meters)
    x0_full[0:3] = np.random.uniform([-0.1, -0.1, 0.14], [0.1, 0.1, 0.16])
    # Orientation bounds (in radians, roughly ±15 degrees)
    x0_full[3:7] = normalize_quaternion(np.array([1.0, 
                                            np.random.uniform(-0.1, 0.1),
                                            np.random.uniform(-0.1, 0.1),
                                            np.random.uniform(-0.1, 0.1)]))
    # Linear velocity bounds (in m/s)
    x0_full[7:10] = np.random.uniform(-0.2, 0.2, 3)
    # Angular velocity bounds (in rad/s)
    x0_full[10:13] = np.random.uniform(-0.5, 0.5, 3)
    
    # Initialize history arrays for 13-dimensional state
    x_hist = np.zeros((n_steps+1, 13))  # Pre-allocate reduced state history
    u_hist = np.zeros((n_steps, 12))    # Pre-allocate control history
    t_hist = np.zeros(n_steps+1)        # Pre-allocate time history
    
    # Initial state setup
    x = x0_full.copy()
    x_hist[0] = x0_full  # Store initial state
    t_hist[0] = 0.0

    # Target state - reduced to 13 states
    x_ref = np.zeros(13)  # Reference for reduced states
    x_ref[1] = .03  # Target y position
    x_ref[2] = 0.18  # Target height matches initial height
    x_ref[3] = 1.0   # Quaternion w = 1 for [0 0 0] Euler angles
    
    # Simple simulation loop
    success = False
    sim_start_time = time()
    try:
        for i in tqdm(range(n_steps)):
            t = i * dt
            logger.debug(f"Step {i}/{n_steps} (t={t:.3f}s)")
            
            # Get optimal control input
            try:
                solve_start = time()
                # Add different noise levels for translation and rotation states
                x_noisy = x.copy()
                
                u, status = controller.solve(x_noisy, x_ref, foot_positions)  # Pass foot positions as parameters
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
            x = rk4_step(x, u, foot_positions, dt, m, I)
            
            # Store data (pre-allocated arrays)
            t_hist[i+1] = t + dt
            x_hist[i+1] = x
            u_hist[i] = u
            
        # After simulation loop, before plotting
        if success:
            print(f"Final output: {u}")
            
            sim_time = time() - sim_start_time
            metrics = calculate_metrics(x_hist, u_hist, x_ref, sim_time)
            log_metrics(metrics, logger)
            
            # Create figure with four subplots
            plt.figure(figsize=(15, 16))  # Made figure taller
            
            # Position plot
            plt.subplot(4, 1, 1)  # Changed to 4x1 layout
            for i, label in enumerate(['x', 'y', 'z']):
                plt.plot(t_hist, x_hist[:, i], label=label)
            plt.grid(True)
            plt.legend()
            plt.title('Position vs Time')
            
            # Orientation plot
            plt.subplot(4, 1, 2)  # Changed to 4x1 layout
            for i, label in enumerate(["w","x","y","z"]):
                plt.plot(t_hist, x_hist[:, i+3], label=label)
            plt.grid(True)
            plt.legend()
            plt.title('Orientation vs Time')
            
            # Foot forces plot
            plt.subplot(4, 1, 3)  # Changed to 4x1 layout
            t_hist_control = t_hist[:-1]  # Control history is one step shorter
            legs = ['FL', 'FR', 'RL', 'RR']
            colors = ['r', 'g', 'b', 'm']
            
            total_force = np.zeros_like(t_hist_control)
            for leg in range(4):
                force_z = u_hist[:, leg*3 + 2]
                plt.plot(t_hist_control, force_z, 
                        label=f'{legs[leg]} Force Z',
                        color=colors[leg])
                total_force += force_z
            
            # Plot total force
            plt.plot(t_hist_control, total_force, 
                    label='Total Force Z',
                    color='k', 
                    linestyle='--',
                    linewidth=2)
            
            plt.grid(True)
            plt.legend()
            plt.title('Vertical Foot Forces vs Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Force (N)')
            
            # New foot position XY plot with position labels (using fixed foot positions)
            plt.subplot(4, 1, 4)
            foot_pos_array = foot_positions.reshape(4, 3)
            legs = ['FL', 'FR', 'RL', 'RR']
            colors = ['r', 'g', 'b', 'm']
            
            # Plot each foot position (fixed positions)
            for i, (foot_pos, leg, color) in enumerate(zip(foot_pos_array, legs, colors)):
                x_pos, y_pos = foot_pos[0], foot_pos[1]
                plt.plot(x_pos, y_pos, 'o', label=f'{leg} Foot', color=color, markersize=8)
                
                # Add position label
                pos_label = f'({x_pos:.3f}, {y_pos:.3f})'
                plt.annotate(pos_label, (x_pos, y_pos), 
                           xytext=(10, 10), textcoords='offset points',
                           color=color, fontsize=8)
            
            plt.grid(True)
            plt.legend()
            plt.title('Fixed Foot Positions in XY Plane')
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.axis('equal')  # Make plot aspect ratio 1:1
            
            plt.tight_layout()
            plt.show()
            
            # Save data
            data_file = os.path.join(log_dir, f'simulation_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.npz')
            np.savez(data_file, 
                    x_hist=x_hist, 
                    u_hist=u_hist, 
                    t_hist=t_hist,
                    x_ref=x_ref,
                    foot_positions=foot_positions,
                    metrics=metrics)
            logger.info(f"Simulation data saved to {data_file}")
            
        else:
            logger.error("No successful iterations to plot")
            
    except Exception as e:
        logger.error(f"Simulation failed: {str(e)}", exc_info=True)
        
    # ...existing error handling code...

if __name__ == "__main__":
    main()