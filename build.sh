export PATH=/usr/bin:/bin:/usr/sbin:/sbin
rm -rf build install log
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash # setup biến môi trường
colcon build --packages-select serial

colcon build --packages-select robot_bridge_node

colcon build --packages-select robot_description
colcon build --packages-select robot_ekf
colcon build --packages-select rplidar_ros
colcon build --packages-select robot_lidar
colcon build --packages-select robot_slam
colcon build --packages-select robot_nav2
colcon build --packages-select robot_gui
colcon build --packages-select robot_human_tracker
# colcon build --symlink-install
source ./install/setup.bash

ros2 pkg prefix robot_lidar
ros2 pkg prefix robot_bridge_node
