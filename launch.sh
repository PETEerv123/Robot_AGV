export PATH=/usr/bin:/bin:/usr/sbin:/sbin

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

source /opt/ros/humble/setup.bash # setup biến môi trường
colcon build --packages-select robot_bringup

source ./install/setup.bash

ros2 pkg prefix robot_bringup

ros2 launch robot_bringup robot.launch.py
