sudo apt-get update


rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y


source /opt/ros/jazzy/setup.bash
colcon build --packages-select zenoh_cpp_vendor rmw_zenoh_cpp
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# colcon build --packages-select zenoh_cpp_vendor
# colcon build --packages-select rmw_zenoh_cpp
source install/setup.bash
# ros2 run rmw_zenoh_cpp rmw_zenohd 
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash



export ZENOH_ROUTER_CONFIG_URI=/home/ws/src/rmw_zenoh/rmw_zenoh_cpp/config/routerconfig.json5

ros2 run rmw_zenoh_cpp rmw_zenohd 
# sudo chmod a+rw /dev/i2c-7
# sudo chmod a+rw /dev/ttyACM0