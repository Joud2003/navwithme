from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_fake_detections = LaunchConfiguration("use_fake_detections")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_detections",
                default_value="false",
                description=("Use synthetic detections instead of YOLO inference"),
            ),
            Node(
                package="robot_controller_cpp",
                executable="robot_controller",
                output="screen",
            ),
            Node(
                package="nav_with_me",
                executable="move_robot",
                parameters=[{"use_fake_detections": use_fake_detections}],
                output="screen",
            ),
        ]
    )
