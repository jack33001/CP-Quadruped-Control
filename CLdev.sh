sudo chmod a+rw /dev/i2c-7
sudo chmod a+rw /dev/ttyUSB0
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# colcon build --packages-select quadruped_hardware
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# ros2 launch quadruped_bringup real.launch.py