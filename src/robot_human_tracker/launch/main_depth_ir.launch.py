from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace="camera",
                name="camera",
                parameters=[
                    {
                        "initial_reset": True,
                        "enable_color": True,
                        "enable_depth": True,
                        "enable_infra1": True,
                        "enable_infra2": False,
                        "depth_module.profile": "640x360x30",
                        "infra_width": 640,
                        "infra_height": 360,
                        "infra_fps": 30,
                        "align_depth.enable": True,
                        "emitter_enabled": False,
                        "depth_module.enable_auto_exposure": True,
                        "enable_gyro": False,
                        "enable_accel": False,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_human_tracker",
                executable="depth_tracker",
                name="depth_tracker_node",
                output="screen",
            ),
            Node(
                package="robot_human_tracker",
                executable="go2_follower",
                name="go2_follower_node",
                output="screen",
            ),
        ]
    )
