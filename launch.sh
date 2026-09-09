export PATH=/usr/bin:/bin:/usr/sbin:/sbin

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

source /opt/ros/humble/setup.bash # setup biến môi trường
colcon build --packages-select robot_bringup

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source ./install/setup.bash

ros2 pkg prefix robot_bringup

# ros2 launch realsense2_camera rs_launch.py
ros2 launch robot_bringup robot.launch.py
