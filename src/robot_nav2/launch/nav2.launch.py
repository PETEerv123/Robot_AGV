import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_robot_nav2 = get_package_share_directory("robot_nav2")

    default_params_file = os.path.join(
        pkg_robot_nav2,
        "config",
        "nav2_params.yaml",
    )

    navigation_launch_file = os.path.join(
        pkg_robot_nav2,
        "launch",
        "navigation_custom_launch.py",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_autostart,
            declare_params_file,
            navigation_launch,
        ]
    )
