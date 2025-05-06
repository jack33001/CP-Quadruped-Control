#!/usr/bin/env python3

import os
import shutil
import logging
import yaml
from optimal_controller import QuadrupedOptimalController

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    # Get package root directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(os.path.dirname(script_dir))
    export_dir = os.path.join(package_root, 'include/quadruped_mpc/acados_generated')
    
    print(f"Generating ACADOS code to: {export_dir}")
    os.makedirs(export_dir, exist_ok=True)
    os.makedirs(os.path.join(export_dir, 'quadruped_ode_model'), exist_ok=True)
    
    # Default to package config directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(os.path.dirname(script_dir))
    param_file = os.path.join(root_dir, 'config', 'optimal_controller.yaml')
    
    with open(param_file, 'r') as f:
        params = yaml.safe_load(f)['optimal_controller']
        N = params.get('stages')
        T = params.get('horizon')
    
    try:
        # Create controller - this will generate code
        controller = QuadrupedOptimalController(
            N, 
            T, 
            code_export_dir=export_dir
        )
        
        print("Code generation successful!")
        return 0
        
    except Exception as e:
        print(f"Error during code generation: {str(e)}")
        return 1

if __name__ == "__main__":
    main()