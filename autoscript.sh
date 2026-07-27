export PATH=/usr/bin:/bin:/usr/sbin:/sbin
rm -rf build install log

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

source /opt/ros/humble/setup.bash # setup biến môi trường
colcon build --packages-select serial

source ./install/setup.bash

colcon build --packages-select robot_bridge_node

source ./install/setup.bash
source ~/.bashrc
