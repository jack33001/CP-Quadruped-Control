#!/usr/bin/env python3

import numpy as np
from optimal_controller import QuadrupedOptimalController
import matplotlib.pyplot as plt
import logging

# Setup logging
logger = logging.getLogger(__name__)

def main():
    # Robot physical parameters
    m = 1.0  # mass in kg
    I = 0.1  # inertia in kg*m^2
    
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
    try:
        for i in range(n_steps):
            t = i * dt
            
            # Get optimal control input
            try:
                u, status = controller.solve(x, x_ref)
                success = True
            except Exception as e:
                logger.error(f"Solver failed at t={t}: {e}")
                break
                
            # Update state (full 12-dim state)
            dx = np.zeros(12)
            dx[0:6] = x[6:12]         # velocities affect positions
            dx[6:9] = u[0:3]/m        # linear accelerations
            dx[9:12] = u[3:6]/I       # angular accelerations
            
            # Euler integration
            x = x + dt * dx
            
            # Store data (pre-allocated arrays)
            t_hist[i+1] = t + dt
            x_hist[i+1] = x
            u_hist[i] = u
            
        # Plot results from numpy arrays directly
        if success:
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