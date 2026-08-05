#!/bin/bash
pkill -9 -f map_saver
pkill -9 -f lifecycle_manager_slam
pkill -9 -f robot_bridge_node
pkill -9 -f robot_lidar
pkill -9 -f robot_state_publisher
pkill -9 -f ekf_node
pkill -9 -f ekf_filter_node
pkill -9 -f slam_toolbox
pkill -9 -f component_container
pkill -9 -f ros2
pkill -9 -f joint_state_publisher
ros2 daemon stop
ros2 daemon start
ros2 node list
sleep 2

echo "ROS stopped."
