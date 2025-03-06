
Debug Build

source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

source /opt/ros/jazzy/setup.bash
source install/setup.bash