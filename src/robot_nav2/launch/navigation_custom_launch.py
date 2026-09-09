import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value="",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("cmd_vel", "/cmd_vel_nav2"),
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": [
                            "controller_server",
                            "planner_server",
                            "smoother_server",
                            "bt_navigator",
                            "behavior_server",
                        ],
                    }
                ],
            ),
        ]
    )
