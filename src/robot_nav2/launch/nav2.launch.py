import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_robot_nav2 = get_package_share_directory('robot_nav2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_params_file = os.path.join(pkg_robot_nav2, 'config', 'nav2_params.yaml')
    nav2_bringup_launch_file = os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_slam = LaunchConfiguration('slam')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Sử dụng thời gian mô phỏng (Gazebo) nếu là true'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Tự động kích hoạt lifecycle nodes của Nav2'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Đường dẫn file cấu hình Nav2 YAML'
    )

    declare_map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Đường dẫn file bản đồ .yaml (Bắt buộc nếu slam=false)'
    )

    declare_use_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Đặt true nếu chạy SLAM (vừa đi vừa dựng bản đồ), false nếu dùng AMCL'
    )

    # 3. Include file bringup_launch.py của hệ thống nav2
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'map': map_yaml_file,
            'slam': use_slam,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_map_yaml_file,
        declare_use_slam,
        nav2_bringup_launch
    ])
