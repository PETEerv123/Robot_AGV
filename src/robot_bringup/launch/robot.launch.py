import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    urdf_file = os.path.join(
        get_package_share_directory("robot_description"),
        "urdf",
        "robot.urdf",
    )

    robot_description = open(urdf_file).read()

    ekf_yaml = os.path.join(
        get_package_share_directory("robot_ekf"),
        "config",
        "ekf.yaml",
    )

    slam_param = os.path.join(
        get_package_share_directory("robot_slam"),
        "config",
        "mapper.yaml",
    )

    nav2_yaml = os.path.join(
        get_package_share_directory("robot_nav2"),
        "config",
        "nav2_params.yaml",
    )

    nav2_launch = os.path.join(
        get_package_share_directory("robot_nav2"),
        "launch",
        "nav2.launch.py",
    )

    twist_mux_yaml = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "twist_mux.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="robot_bridge_node",
                executable="robot_bridge_node",
                name="robot_bridge_node",
                output="screen",
            ),
            Node(
                package="robot_lidar",
                executable="robot_lidar",
                name="robot_lidar",
                output="screen",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_param, {"use_sim_time": False}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_yaml],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "slam": "False",
                    "use_sim_time": "False",
                    "params_file": nav2_yaml,
                    "autostart": "True",
                }.items(),
            ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                output="screen",
                parameters=[twist_mux_yaml],
                remappings=[
                    ("cmd_vel_out", "/cmd_vel"),
                ],
            ),
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="screen",
            # ),
        ]
    )
