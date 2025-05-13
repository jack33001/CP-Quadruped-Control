# sudo chmod a+rw /dev/ttyACM0
# sudo chmod a+rw /dev/ttyUSB0
sudo chmod a+rw /dev/i2c-7
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source install/setup.bash
