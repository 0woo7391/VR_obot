import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 경로 및 설정 준비
    vrobot_description_share = get_package_share_directory('vrobot_description')
    vrobot_moveit_config_share = get_package_share_directory('vrobot_moveit_config')
    
    xacro_file = os.path.join(vrobot_description_share, 'urdf', 'vrobot.urdf.xacro')
    controllers_config = os.path.join(vrobot_description_share, 'config', 'ros2_controllers.yaml')
    
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    declare_use_fake_hardware = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use mock ros2_control hardware backend',
    )

    robot_description_raw = Command([
        'xacro ',
        xacro_file,
        ' use_fake_hardware:=',
        use_fake_hardware,
    ])
    robot_description = {'robot_description': robot_description_raw}

    # 2. 로봇 상태 퍼블리셔 (TF)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # 3. 제어기 매니저 (Hardware Interface)
    node_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_config],
        output='both',
    )

    # 4. 제어기 스포너 (순차 실행)
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '100'],
        output='screen',
    )

    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['doosan_arm_controller', '--controller-manager-timeout', '100'],
        output='screen',
    )

    spawn_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager-timeout', '100'],
        output='screen',
    )

    # 5. MoveIt2 설정 (demo.launch.py 로직 활용)
    moveit_config = MoveItConfigsBuilder("vrobot", package_name="vrobot_moveit_config")\
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl", "chomp"])\
        .to_moveit_configs()

    # Move Group 노드
    node_move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    node_static_collision_scene = Node(
        package='vrobot_description',
        executable='add_static_collision_scene.py',
        output='screen',
    )

    # RViz2 노드
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        arguments=['-d', os.path.join(vrobot_moveit_config_share, 'config', 'moveit.rviz')],
    )

    # 6. 이벤트 핸들러를 통한 순차 실행 제어
    # 제어기 매니저 시작 후 JSB 스폰
    load_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_controller_manager,
            on_start=[spawn_jsb],
        )
    )

    # JSB 스폰 후 Arm 제어기 스폰
    load_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_arm],
        )
    )

    # Arm 제어기 스폰 후 Gripper 제어기 스폰
    load_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_arm,
            on_exit=[spawn_gripper],
        )
    )

    return LaunchDescription([
        declare_use_fake_hardware,
        node_robot_state_publisher,
        node_controller_manager,
        load_jsb,
        load_arm,
        load_gripper,
        node_move_group,
        node_static_collision_scene,
        node_rviz,
    ])
