import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # 신설한 통합 패키지의 공유 디렉토리 경로 확보
    pkg_share = get_package_share_directory('vrobot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'vrobot.urdf.xacro')

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    declare_use_fake_hardware = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use mock ros2_control hardware backend',
    )

    # 시스템 xacro 유틸리티를 이용해 조립 뼈대 파싱
    robot_description_raw = Command([
        'xacro ',
        xacro_file,
        ' use_fake_hardware:=',
        use_fake_hardware,
    ])

    # 1. Robot State Publisher Node (3D 좌표 고정 및 변환 브로드캐스팅)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # 2. Joint State Publisher GUI Node (관절 제어용 마우스 슬라이더 컴포넌트)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # 3. RViz2 시각화 메인 엔진 구동
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        declare_use_fake_hardware,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
