# /bin/python3 /home/ws/src/quadruped_mpc/scripts/acados/generate_controller.py

colcon build --symlink-install --packages-skip quadruped_hardware quadruped_utils imu_hardware_interface

source install/setup.bash

ros2 launch quadruped_bringup gz.launch.py