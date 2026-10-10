from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_controller_cpp",
                executable="robot_controller",
                output="screen",
            ),
            Node(
                package="nav_with_me",
                executable="move_robot",
                output="screen",
            ),
        ]
    )
