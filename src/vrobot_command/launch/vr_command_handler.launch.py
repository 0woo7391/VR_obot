import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("vrobot_command"),
        "config",
        "vr_command_handler.yaml",
    )

    return LaunchDescription([
        Node(
            package="vrobot_command",
            executable="vr_command_handler.py",
            name="vr_command_handler",
            parameters=[config_file],
            output="screen",
        ),
    ])
