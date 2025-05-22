#!/usr/bin/env python3
import sys
import os
import logging
import argparse
from optimal_controller import QuadrupedOptimalController

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    parser = argparse.ArgumentParser(description="Generate ACADOS controller for quadruped robot")
    parser.add_argument('--output-dir', type=str, help='Directory to output generated files')
    parser.add_argument('--config', type=str, help='Path to configuration file')
    args = parser.parse_args()

    # Set output directory
    if args.output_dir:
        output_dir = args.output_dir
    else:
        # Default to include directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        root_dir = os.path.dirname(os.path.dirname(script_dir))
        output_dir = os.path.join(root_dir, 'include', 'quadruped_mpc', 'acados_generated')
    
    logger.info(f"Generating ACADOS code to: {output_dir}")
    
    # Use the config file from the argument or the default one
    config_file = args.config
    if not config_file:
        # Use the consolidated config file instead of optimal_controller.yaml
        root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        config_file = os.path.join(root_dir,'config', 'quadruped_controllers.yaml')
    
    # Create controller
    controller = QuadrupedOptimalController(code_export_dir=output_dir, param_file=config_file)
    logger.info("Code generation successful!")

if __name__ == "__main__":
    main()