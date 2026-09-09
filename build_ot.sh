export PATH=/usr/bin:/bin:/usr/sbin:/sbin

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

rm -rf ./build/robot_human_tracker
source /opt/ros/humble/setup.bash # setup biến môi trường

colcon build --packages-select robot_human_tracker

source ./install/setup.bash && ros2 launch robot_human_tracker main_depth_ir.launch.py
