import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share = get_package_share_directory('vrobot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'vrobot.urdf.xacro')
    controllers_config = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    declare_use_fake_hardware = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use mock ros2_control hardware backend',
    )

    # Robot Description
    robot_description_raw = Command([
        'xacro ',
        xacro_file,
        ' use_fake_hardware:=',
        use_fake_hardware,
    ])
    robot_description = {'robot_description': robot_description_raw}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Controller Manager
    node_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_config],
        output='both',
    )

    # Spawn Broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Spawn Arm Controller
    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['doosan_arm_controller'],
        output='screen',
    )

    # Spawn Gripper Controller
    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    return LaunchDescription([
        declare_use_fake_hardware,
        node_robot_state_publisher,
        node_controller_manager,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller,
    ])
