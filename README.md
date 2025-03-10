
Debug Build

source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

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




cd src
git clone https://github.com/ros2/rmw_zenoh.git -b jazzy
cd ~/ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash


git clone https://github.com/ros2/rmw_zenoh.git -b jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release