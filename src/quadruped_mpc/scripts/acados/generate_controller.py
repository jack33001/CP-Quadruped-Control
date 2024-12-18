#!/usr/bin/env python3

import os
import shutil
import logging
from optimal_controller import QuadrupedOptimalController

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    # Set export directory
    export_dir = os.path.join(os.path.dirname(__file__), '../acados_generated')
    os.makedirs(export_dir, exist_ok=True)
    
    try:
        # Create controller - this will generate code
        controller = QuadrupedOptimalController(
            N=20, 
            T=0.2, 
            code_export_dir=export_dir
        )
        
        # Copy generated files from temp location if needed
        c_generated = "c_generated_code"
        if os.path.exists(c_generated):
            for item in os.listdir(c_generated):
                src = os.path.join(c_generated, item)
                dst = os.path.join(export_dir, item)
                if os.path.isfile(src):
                    shutil.copy2(src, dst)
                elif os.path.isdir(src):
                    shutil.copytree(src, dst, dirs_exist_ok=True)
            shutil.rmtree(c_generated)
            
        print("Code generation successful!")
        return 0
        
    except Exception as e:
        print(f"Error: {str(e)}")
        return 1

if __name__ == "__main__":
    main()