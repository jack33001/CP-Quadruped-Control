
Debug Build

source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /opt/ros/jazzy/setup.bash
source install/setup.bash


Zenoh

sudo apt update && sudo apt install ros-jazzy-rmw-zenoh-cpp 
rm -rf build/ install/ log/
source /opt/ros/jazzy/setup.bash
pkill -9 -f ros && ros2 daemon stop
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd 


<!-- sudo apt update && sudo apt install ros-jazzy-rmw-zenoh-cpp -->
rm -rf build/ install/ log/
pkill -9 -f ros && ros2 daemon stop
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd


sudo apt update && sudo apt install ros-jazzy-rmw-zenoh-cpp 
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd 


__________Zenoh Setup___________

cd src
git clone https://github.com/ros2/rmw_zenoh.git -b jazzy
cd ..
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

source /opt/ros/jazzy/setup.bash
colcon build --packages-select zenoh_cpp_vendor
colcon build --packages-select rmw_zenoh_cpp
source install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash


<<<<<<< HEAD
export ZENOH_ROUTER_CONFIG_URI=$HOME/ws/routerconfig.json5 
=======
export ZENOH_ROUTER_CONFIG_URI=/home/ws/src/rmw_zenoh/rmw_zenoh_cpp/config/routerconfig.json5

ros2 run rmw_zenoh_cpp rmw_zenohd 
>>>>>>> fbd533b (some hardware interface issues Working on imu)
________________________________

git clone https://github.com/ros2/rmw_zenoh.git -b jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

sudo netstat -tupn | grep -e 205 -e 127.0.0.1