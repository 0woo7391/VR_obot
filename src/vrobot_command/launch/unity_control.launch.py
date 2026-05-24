import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("vrobot_command"),
        "config",
        "vr_command_handler.yaml",
    )

    ros_ip = LaunchConfiguration("ros_ip")
    ros_tcp_port = LaunchConfiguration("ros_tcp_port")
    execute_enabled = LaunchConfiguration("execute_enabled")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ros_ip",
            default_value="0.0.0.0",
            description="IP address for ros_tcp_endpoint to bind.",
        ),
        DeclareLaunchArgument(
            "ros_tcp_port",
            default_value="10000",
            description="TCP port for Unity ROS-TCP-Connector.",
        ),
        DeclareLaunchArgument(
            "execute_enabled",
            default_value="false",
            description="Whether vr_command_handler executes planned arm trajectories.",
        ),
        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            name="ros_tcp_endpoint",
            parameters=[
                {"ROS_IP": ros_ip},
                {"ROS_TCP_PORT": ros_tcp_port},
            ],
            output="screen",
        ),
        Node(
            package="vrobot_command",
            executable="vr_command_handler.py",
            name="vr_command_handler",
            parameters=[
                config_file,
                {"execute_enabled": execute_enabled},
            ],
            output="screen",
        ),
    ])
