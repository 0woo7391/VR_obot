# E0509 ROS 2 AI 제어 명세

이 문서는 `doosan-robot2` 저장소의 E0509 모델을 ROS 2에서 제어하기 위해 필요한 launch, endpoint, 단위, QoS, 제약 사항만 정리한 AI 입력용 명세다.

기준 정보:

- 저장소 루트: `doosan-robot2`
- ROS 2 배포판: Humble 기준
- 주 모델: `e0509`
- 기본 관절명: `joint_1` ~ `joint_6`
- MoveIt group: `manipulator`
- MoveIt chain: `base_link` -> `link_6`

## 1. 절대 규칙

1. `dsr_controller2`가 만드는 서비스와 토픽은 상대 이름이다. launch에서 `name:=dsr01`이면 앞에 `/dsr01/`이 붙고, `name:=""`이면 루트(`/`) 아래에 생긴다.
2. DSR motion 서비스(`MoveJoint`, `MoveLine`)는 Doosan API 단위를 따른다.
   - 관절각: degree
   - TCP 위치: mm
   - TCP 자세: degree
3. MoveIt과 `FollowJointTrajectory`는 일반 ROS 관례를 따른다.
   - 관절 위치: rad
   - trajectory time: sec/nsec
4. `e0509_gripper_gazebo.launch.py`는 일반 `dsr_controller2` motion 서비스를 띄우지 않는다. 이 launch만 켠 상태에서 `/motion/move_joint`를 기대하지 말 것.
5. Gazebo Classic 그리퍼 명령은 `dsr_msgs2/srv/gripper/*` 서비스가 아니라 `/gripper_controller/commands` 토픽이다.
6. namespace 기본값은 launch별로 다르므로 실행 중인 endpoint를 확인한다.

## 2. 빠른 제어 참조

| 목적 | 이름 | 타입 | QoS / 단위 / 주의사항 |
| --- | --- | --- | --- |
| 단발 관절 이동 | `/<ns>/motion/move_joint` | `dsr_msgs2/srv/MoveJoint` | service 기본 QoS Reliable/Volatile; pos degree, vel deg/s, acc deg/s2, time sec, radius mm |
| 단발 TCP 직선 이동 | `/<ns>/motion/move_line` | `dsr_msgs2/srv/MoveLine` | service 기본 QoS Reliable/Volatile; pos `[x,y,z,rx,ry,rz]` = mm/mm/mm/degree/degree/degree, vel `[mm/s, deg/s]`, acc `[mm/s2, deg/s2]`, radius mm |
| MoveIt trajectory 실행 | `/<ns>/dsr_moveit_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | action 기본 QoS; joint position rad, velocity rad/s, time sec/nsec |
| Gazebo 그리퍼 위치 | `/gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | controller 기본 QoS; `rh_r1` position command |
| 현재 joint 조회 | `/<ns>/aux_control/get_current_posj` | `dsr_msgs2/srv/GetCurrentPosj` | service 기본 QoS Reliable/Volatile; joint position degree |
| 현재 TCP 조회 | `/<ns>/aux_control/get_current_posx` | `dsr_msgs2/srv/GetCurrentPosx` | service 기본 QoS Reliable/Volatile; TCP pose mm, degree |
| motion 완료 대기 | `/<ns>/motion/move_wait` | `dsr_msgs2/srv/MoveWait` | service 기본 QoS Reliable/Volatile; unit 없음 |
| motion 정지 | `/<ns>/motion/move_stop` | `dsr_msgs2/srv/MoveStop` | service 기본 QoS Reliable/Volatile; `stop_mode` enum |

`<ns>` 처리 규칙:

- namespace가 `dsr01`이면 `/dsr01/motion/move_joint`
- namespace가 빈 문자열이면 `/motion/move_joint`
- endpoint를 사용할 때 중복 slash가 생기지 않도록 한다.

## 2.1 통합 패키지 분석 요약

이 루트 문서는 E0509를 실제로 제어할 때 필요한 Doosan ROS 2 패키지를 통합해서 본다. 패키지별 별도 `_manual.md`를 만들지 않는다.

| 패키지 | 역할 | 핵심 파일 |
| --- | --- | --- |
| `dsr_controller2` | DRFL API를 ROS 2 service/topic/action 및 ros2_control controller로 노출 | `src/dsr_controller2.cpp`, `src/dsr_joint_trajectory.cpp`, `config/dsr_controller2.yaml` |
| `dsr_hardware2` | DRCF/DRFL 연결을 ros2_control `SystemInterface`로 제공 | `src/dsr_hw_interface2.cpp` |
| `dsr_msgs2` | Doosan custom msg/srv/action 타입 정의 | `msg/*`, `srv/*`, `action/*` |
| `dsr_description2` | E0509 URDF/Xacro, ros2_control Xacro, mesh | `urdf/e0509.urdf`, `xacro/e0509.urdf.xacro`, `ros2_control/e0509.*.xacro` |
| `dsr_moveit_config_e0509` | E0509 MoveIt 2 planning/controller/limit 설정 | `config/joint_limits.yaml`, `config/moveit_controllers.yaml`, `launch/start.launch.py` |
| `dsr_bringup2` | MoveIt/Gazebo/MuJoCo/그리퍼 bringup 조합 | `launch/dsr_bringup2_*.launch.py`, `launch/e0509_gripper_gazebo.launch.py` |
| `dsr_gazebo2`, `dsr_mujoco` | simulation controller spawn 및 command relay | 각 `launch/*.launch.py`, `config/*.yaml` |

`dsr_hardware2`의 실제 단위 변환은 코드 기준이다. real mode read는 `read_data_rt()`의 joint degree 값을 rad로 바꿔 state interface에 넣고, virtual mode read는 `GetCurrentPose()` joint degree 값을 rad로 바꾼다. write는 command interface의 rad/rad/s를 degree/degree/s로 바꿔 real mode에서는 `servoj_rt`, virtual mode에서는 `amovej`를 호출한다.

## 3. Launch별 namespace와 기능

| Launch | 기본 namespace | `dsr_controller2` 서비스 | MoveIt action | 그리퍼 토픽 |
| --- | --- | --- | --- | --- |
| `dsr_moveit2/dsr_moveit_config_e0509/launch/start.launch.py` | `""` | 있음 | 있음 | 없음 |
| `dsr_bringup2/launch/dsr_bringup2_moveit.launch.py model:=e0509` | `""` | 있음 | 있음 | 없음 |
| `dsr_bringup2/launch/dsr_bringup2_gazebo.launch.py model:=e0509` | `dsr01` | 있음 | 없음 | 없음 |
| `dsr_moveit2/dsr_moveit_config_e0509/launch/demo.launch.py` | 대부분 없음 | 기본적으로 없음 | 있음 | 없음 |
| `dsr_bringup2/launch/e0509_gripper_gazebo.launch.py` | 없음 | 없음 | 없음 | `/gripper_controller/commands` |

Launch 실행 명령:

```bash
ros2 launch dsr_moveit_config_e0509 start.launch.py mode:=virtual model:=e0509 name:=
ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py mode:=virtual model:=e0509 name:=dsr01
ros2 launch dsr_bringup2 e0509_gripper_gazebo.launch.py
```

## 4. 필수 요청 페이로드

이 섹션은 주요 service/topic/action 요청 payload 예시를 정리한다. 전체 endpoint 목록과 QoS 상세는 6~7절을 기준으로 한다.

### 4.1 `MoveJoint` service

파일: `dsr_msgs2/srv/motion/MoveJoint.srv`

```text
float64[6] pos        # target joint angle [degree]
float64 vel           # [deg/sec]
float64 acc           # [deg/sec2]
float64 time          # [sec], 0이면 vel/acc 사용
float64 radius        # [mm]
int8 mode             # 0 absolute, 1 relative
int8 blend_type       # 0 duplicate, 1 override
int8 sync_type        # 0 sync, 1 async
---
bool success
```

예시:

```bash
ros2 service call /motion/move_joint dsr_msgs2/srv/MoveJoint \
"{pos: [0, 0, 90, 0, 90, 0], vel: 30.0, acc: 60.0, time: 0.0, radius: 0.0, mode: 0, blend_type: 0, sync_type: 0}"
```

주의사항:

- `pos`는 항상 6개인지 검사한다.
- DSR 서비스에 보낼 때 rad로 변환하지 않는다.
- MoveIt에서 얻은 rad 값을 DSR `MoveJoint`로 보낼 때만 degree로 변환한다.
- 동기 이동이 필요하면 `sync_type=0`을 사용하고, 완료 확인은 `move_wait` 또는 상태 조회로 한다.

### 4.2 `MoveLine` service

파일: `dsr_msgs2/srv/motion/MoveLine.srv`

```text
float64[6] pos        # [x, y, z, a, b, c], mm/degree
float64[2] vel        # [mm/sec, deg/sec]
float64[2] acc        # [mm/sec2, deg/sec2]
float64 time          # [sec]
float64 radius        # [mm]
int8 ref              # 0 base, 1 tool, 2 world
int8 mode             # 0 absolute, 1 relative
int8 blend_type       # 0 duplicate, 1 override
int8 sync_type        # 0 sync, 1 async
---
bool success
```

예시:

```bash
ros2 service call /motion/move_line dsr_msgs2/srv/MoveLine \
"{pos: [400, 0, 500, 0, 180, 0], vel: [100.0, 30.0], acc: [200.0, 60.0], time: 0.0, radius: 0.0, ref: 0, mode: 0, blend_type: 0, sync_type: 0}"
```

주의사항:

- `pos`는 `[x, y, z, a, b, c]` 6개다.
- `vel`과 `acc`는 각각 2개다.
- `ref=2`는 controller/firmware 조건에 따라 제한될 수 있으므로 기본은 `ref=0`이다.

### 4.3 MoveIt `FollowJointTrajectory` action

MoveIt 설정 파일: `dsr_moveit2/dsr_moveit_config_e0509/config/moveit_controllers.yaml`

액션 이름:

- namespace 없음: `/dsr_moveit_controller/follow_joint_trajectory`
- namespace `dsr01`: `/dsr01/dsr_moveit_controller/follow_joint_trajectory`

controller joints:

```text
joint_1
joint_2
joint_3
joint_4
joint_5
joint_6
```

주의사항:

- `JointTrajectory.joint_names` 순서를 위 순서로 고정한다.
- position 값은 rad다.
- MoveIt planning scene이나 `move_group`을 사용할 때는 group name `manipulator`를 쓴다.
- DSR `MoveJoint` 서비스와 혼합할 때 단위 변환 위치를 명확히 분리한다.

### 4.4 Gazebo Classic RH-P12 gripper topic

Launch: `dsr_bringup2/launch/e0509_gripper_gazebo.launch.py`

명령 토픽:

```text
/gripper_controller/commands
std_msgs/msg/Float64MultiArray
```

예시:

```bash
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [0.5]}"
```

주의사항:

- 이 launch는 namespace를 쓰지 않는다.
- `rh_r1` 하나를 position command로 제어하는 구조다.
- 이 launch에서는 arm motion service가 없으므로 팔 제어가 필요하면 별도 bringup 또는 controller 구성을 추가해야 한다.

## 5. 주요 노드와 controller

| 이름 | 패키지/타입 | 역할 |
| --- | --- | --- |
| `run_emulator` | `dsr_bringup2` executable | virtual mode Doosan emulator |
| `ros2_control_node` | `controller_manager` | controller manager |
| `robot_state_publisher` | `robot_state_publisher` | TF, robot description |
| `joint_state_broadcaster` | controller | `joint_states` 발행 |
| `dsr_controller2` | `dsr_controller2/RobotController` | Doosan motion/system/IO/realtime 서비스 |
| `dsr_moveit_controller` | `joint_trajectory_controller/JointTrajectoryController` | MoveIt trajectory 실행 |
| `gazebo_connection` | `dsr_bringup2` executable | Gazebo joint state 정렬, command relay |
| `move_group` | `moveit_ros_move_group` | MoveIt planning server |

`start.launch.py`가 spawn하는 controller:

```text
joint_state_broadcaster
dsr_controller2
dsr_moveit_controller
```

`dsr_bringup2_gazebo.launch.py`가 spawn하는 controller:

```text
joint_state_broadcaster
dsr_controller2
```

`e0509_gripper_gazebo.launch.py`가 spawn하는 controller:

```text
joint_state_broadcaster
effort_controller
gripper_controller
```

## 6. 전체 서비스 그룹

`dsr_controller2/src/dsr_controller2.cpp`에서 생성되는 서비스 그룹이다. 자세한 필드는 `ros2 interface show dsr_msgs2/srv/<Type>`로 확인한다.

| 그룹 | 대표 서비스 | 용도 |
| --- | --- | --- |
| `system/*` | `get_robot_state`, `set_robot_control`, `servo_off` | robot mode/state/control |
| `motion/*` | `move_joint`, `move_line`, `move_stop`, `move_wait`, `fkin`, `ikin` | 이동, 정지, 좌표 계산 |
| `aux_control/*` | `get_current_posj`, `get_current_posx`, `get_joint_torque` | 상태 조회 |
| `force/*` | `task_compliance_ctrl`, `set_desired_force`, `release_force` | compliance/force |
| `io/*` | `set_ctrl_box_digital_output`, `get_tool_digital_input` | control box/tool IO |
| `modbus/*` | `config_create_modbus`, `get_modbus_input`, `set_modbus_output` | Modbus |
| `tcp/*` | `set_current_tcp`, `get_current_tcp` | TCP 설정 |
| `tool/*` | `set_current_tool`, `get_current_tool` | tool 설정 |
| `drl/*` | `drl_start`, `drl_stop`, `get_drl_state` | DRL 실행 |
| `realtime/*` | `connect_rt_control`, `start_rt_control`, `read_data_rt` | RT 제어 |
| `plc/*` | `get_input_register_int`, `set_output_register_int` | PLC register |

중요: `dsr_msgs2`에는 gripper 서비스 타입이 있지만, 현재 `dsr_controller2`가 해당 gripper 서비스를 create_service로 등록하지 않는다. gripper 서비스 서버가 필요하면 별도 노드를 작성해야 한다.

## 7. 전체 토픽과 액션

`dsr_controller2` 기준 상대 토픽:

| 이름 | 타입 | 방향 | QoS / 단위 / 주의사항 |
| --- | --- | --- | --- |
| `joint_states` | `sensor_msgs/msg/JointState` | publish | broadcaster/controller 기본 QoS; position rad, velocity rad/s |
| `error` | `dsr_msgs2/msg/RobotError` | publish | `create_publisher(..., 100)`: KeepLast(100) Reliable/Volatile; DRFL error payload, unit 없음 |
| `robot_disconnection` | `dsr_msgs2/msg/RobotDisconnection` | publish | `create_publisher(..., 100)`: KeepLast(100) Reliable/Volatile; disconnection signal, unit 없음 |
| `io/ctrl_box_digital_input_state` | `std_msgs/msg/UInt8MultiArray` | publish | `SystemDefaultsQoS`; data[0..15]=DI[1..16], data[16..31]=DO[1..16], 값 0/1, 10 Hz timer |
| `/rt_topic/<key>` | `std_msgs/msg/Float32MultiArray` | publish | `SystemDefaultsQoS`; `use_rt_topic_pub=true`일 때 `rt_topic_keys`마다 생성, 주기 `rt_timer_ms` ms |
| `alter_motion_stream` | `dsr_msgs2/msg/AlterMotionStream` | subscribe | KeepLast(20) Reliable/Volatile; `pos[6]`, msg 주석에 단위 미기재 |
| `servoj_stream` | `dsr_msgs2/msg/ServojStream` | subscribe | KeepLast(20) Reliable/Volatile; pos degree, vel deg/s, acc deg/s2, time sec |
| `servol_stream` | `dsr_msgs2/msg/ServolStream` | subscribe | KeepLast(20) Reliable/Volatile; pos mm/degree, vel `[mm/s, deg/s]`, acc `[mm/s2, deg/s2]`, time sec |
| `speedj_stream` | `dsr_msgs2/msg/SpeedjStream` | subscribe | KeepLast(20) Reliable/Volatile; vel deg/s, acc deg/s2, time sec |
| `speedl_stream` | `dsr_msgs2/msg/SpeedlStream` | subscribe | KeepLast(10) Reliable/Volatile; vel `[mm/s, mm/s, mm/s, deg/s, deg/s, deg/s]`, acc `[mm/s2, deg/s2]`, time sec |
| `servoj_rt_stream` | `dsr_msgs2/msg/ServojRtStream` | subscribe | KeepLast(20) Reliable/Volatile; pos degree, vel deg/s, acc deg/s2, time sec |
| `servol_rt_stream` | `dsr_msgs2/msg/ServolRtStream` | subscribe | KeepLast(20) Reliable/Volatile; pos mm/degree, vel `[mm/s, deg/s]`, acc `[mm/s2, deg/s2]`, time sec |
| `speedj_rt_stream` | `dsr_msgs2/msg/SpeedjRtStream` | subscribe | KeepLast(20) Reliable/Volatile; vel deg/s, acc deg/s2, time sec |
| `speedl_rt_stream` | `dsr_msgs2/msg/SpeedlRtStream` | subscribe | KeepLast(20) Reliable/Volatile; vel `[mm/s, mm/s, mm/s, deg/s, deg/s, deg/s]`, acc `[mm/s2, deg/s2]`, time sec |
| `torque_rt_stream` | `dsr_msgs2/msg/TorqueRtStream` | subscribe | KeepLast(20) Reliable/Volatile; `tor[6]`, msg 주석에 torque 물리 단위 미기재, time sec |

기타 controller 토픽:

| 이름 | 타입 | QoS / 단위 / 용도 |
| --- | --- | --- |
| `dsr_moveit_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | controller 기본 QoS; position rad, velocity rad/s, trajectory time sec/nsec |
| `dsr_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | controller 기본 QoS; forward position command, joint position rad |
| `/gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | controller 기본 QoS; RH-P12 gripper `rh_r1` position command |
| `/effort_controller/commands` | `std_msgs/msg/Float64MultiArray` | controller 기본 QoS; E0509+gripper Gazebo arm effort command, effort 단위는 controller/URDF 설정 기준 |

액션:

| 이름 | 타입 | QoS / 단위 / 상태 |
| --- | --- | --- |
| `motion/movej_h2r` | `dsr_msgs2/action/MovejH2r` | action 기본 QoS; target_pos degree, target_vel deg/s, target_acc deg/s2; `dsr_controller2` 등록 |
| `motion/movel_h2r` | `dsr_msgs2/action/MovelH2r` | action 기본 QoS; target_pos mm/degree, target_vel `[mm/s, deg/s]`, target_acc `[mm/s2, deg/s2]`; `dsr_controller2` 등록 |
| `dsr_moveit_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | action 기본 QoS; trajectory position rad, velocity rad/s; MoveIt 실행 |

H2R action 예외 처리 기준:

- goal 수락 후 `Drfl->movej_h2r()` 또는 `Drfl->movel_h2r()`가 false이면 action을 abort한다.
- cancel 요청이 오면 `Drfl->stop(STOP_TYPE_QUICK)` 호출 후 canceled result를 반환한다.
- `movej_h2r` 도착 판정 허용 오차는 각 joint 0.1이다.
- `movel_h2r` 도착 판정 허용 오차는 각 task pose field 0.3이다.

## 8. E0509 모델 제한

URDF 제한: `dsr_description2/urdf/e0509.urdf`

| joint | lower rad | upper rad | velocity rad/s |
| --- | ---: | ---: | ---: |
| `joint_1` | -6.2832 | 6.2832 | 3.1416 |
| `joint_2` | -6.2832 | 6.2832 | 3.1416 |
| `joint_3` | -2.7053 | 2.7053 | 3.1416 |
| `joint_4` | -6.2832 | 6.2832 | 3.927 |
| `joint_5` | -6.2832 | 6.2832 | 3.927 |
| `joint_6` | -6.2832 | 6.2832 | 3.927 |

MoveIt override: `dsr_moveit2/dsr_moveit_config_e0509/config/joint_limits.yaml`

| joint | min rad | max rad | max velocity rad/s |
| --- | ---: | ---: | ---: |
| `joint_1` | -3.14 | 3.14 | 2.0944 |
| `joint_2` | -1.6581 | 1.6581 | 2.0944 |
| `joint_3` | -2.3562 | 2.3562 | 2.618 |
| `joint_4` | -3.14 | 3.14 | 3.927 |
| `joint_5` | -2.3562 | 2.3562 | 3.927 |
| `joint_6` | -3.14 | 3.14 | 3.927 |

주의사항:

- MoveIt 계획에는 MoveIt override 제한을 우선 적용한다.
- DSR motion service에 직접 넣는 값은 degree 기준으로 제한 검사를 따로 수행한다.
- 실제 로봇에서는 처음 이동을 작은 범위에서 시작한다.

## 9. 코드 근거 파일

이 문서의 endpoint와 타입은 아래 파일을 기준으로 확인했다.

- `dsr_controller2/src/dsr_controller2.cpp`
- `dsr_controller2/src/dsr_joint_trajectory.cpp`
- `dsr_controller2/config/dsr_controller2.yaml`
- `dsr_bringup2/config/dsr_controller2.yaml`
- `dsr_bringup2/launch/dsr_bringup2_gazebo.launch.py`
- `dsr_bringup2/launch/dsr_bringup2_moveit.launch.py`
- `dsr_bringup2/launch/e0509_gripper_gazebo.launch.py`
- `dsr_moveit2/dsr_moveit_config_e0509/launch/start.launch.py`
- `dsr_moveit2/dsr_moveit_config_e0509/launch/demo.launch.py`
- `dsr_moveit2/dsr_moveit_config_e0509/config/moveit_controllers.yaml`
- `dsr_moveit2/dsr_moveit_config_e0509/config/joint_limits.yaml`
- `dsr_msgs2/srv/motion/MoveJoint.srv`
- `dsr_msgs2/srv/motion/MoveLine.srv`

## 10. 현재 코드상 주의점

1. `e0509_gripper_gazebo.launch.py`는 특정 install 경로에 의존하는 world path를 포함하므로, 배포 시 `get_package_share_directory()` 기반 경로로 교체해야 한다.
2. `dsr_bringup2/config/dsr_controller2.yaml`의 `gripper_controller.type` 위치는 spawner 실패 시 확인해야 한다.
3. `demo.launch.py`에는 `robot_controller_spawner`가 정의되어 있지만 return list에서 빠져 있어 `dsr_controller2`가 기본적으로 뜨지 않는다.
4. `moveit_connection.py`는 `/monitored_planning_scene`를 보고 `move_joint`를 호출하는 보조 노드다. 일반 MoveIt 실행 경로는 `follow_joint_trajectory` action이다.
5. `dsr_joint_trajectory.cpp`의 커스텀 보간 controller는 핵심 제어 경로로 사용하기 전에 동작 특성을 확인해야 한다.
6. `dsr_hardware2`는 hardware parameter가 정확히 6개(`host`, `rt_host`, `port`, `mode`, `model`, `update_rate`)가 아니면 `CallbackReturn::ERROR`를 반환한다.
7. `dsr_hardware2`는 hardware joint 수가 6개가 아니면 초기화 실패 처리한다.
8. DRCF 연결은 20회, 500 ms 간격으로 재시도한다. 권한 획득과 standby 확인은 최대 10초 동안 force request 및 servo on을 반복한다.
9. real/virtual mode 모두 write 주기가 기대 주기 0.3~1.5배 밖이면 해당 write cycle을 skip한다.
10. DRCF version이 M2.12 미만이면 경고를 출력한다. 코드 주석상 M2.12 미만은 더 이상 지원하지 않는다.

## 11. dsr_controller2 사용 인터페이스 원문

이 절은 `dsr_controller2.cpp`에서 실제 `create_service`, `create_subscription`, `create_publisher`, `create_server`로 등록한 `dsr_msgs2` 커스텀 인터페이스만 원문 필드 기준으로 정리한다. 외부 표준 타입의 필드는 ROS 2 표준 정의를 따른다.

공통 QoS: service는 `rmw_qos_profile_services_default`이며 Reliable/Volatile 서비스 QoS이다. `motion/move_stop`만 명시적으로 같은 profile과 callback group을 지정한다. 아래 subscription/publisher는 코드의 depth를 함께 적는다.

### 11.1 등록 endpoint와 타입

| 분류 | 엔드포인트 | 타입 | QoS |
|---|---|---|---|
| Srv | `system/set_robot_mode` | `dsr_msgs2/srv/SetRobotMode` | Reliable / Volatile / service default |
| Srv | `system/get_robot_mode` | `dsr_msgs2/srv/GetRobotMode` | Reliable / Volatile / service default |
| Srv | `system/set_robot_system` | `dsr_msgs2/srv/SetRobotSystem` | Reliable / Volatile / service default |
| Srv | `system/get_robot_system` | `dsr_msgs2/srv/GetRobotSystem` | Reliable / Volatile / service default |
| Srv | `system/get_robot_state` | `dsr_msgs2/srv/GetRobotState` | Reliable / Volatile / service default |
| Srv | `system/set_robot_speed_mode` | `dsr_msgs2/srv/SetRobotSpeedMode` | Reliable / Volatile / service default |
| Srv | `system/get_robot_speed_mode` | `dsr_msgs2/srv/GetRobotSpeedMode` | Reliable / Volatile / service default |
| Srv | `system/get_current_pose` | `dsr_msgs2/srv/GetCurrentPose` | Reliable / Volatile / service default |
| Srv | `system/set_safe_stop_reset_type` | `dsr_msgs2/srv/SetSafeStopResetType` | Reliable / Volatile / service default |
| Srv | `system/get_last_alarm` | `dsr_msgs2/srv/GetLastAlarm` | Reliable / Volatile / service default |
| Srv | `system/servo_off` | `dsr_msgs2/srv/ServoOff` | Reliable / Volatile / service default |
| Srv | `system/set_robot_control` | `dsr_msgs2/srv/SetRobotControl` | Reliable / Volatile / service default |
| Srv | `system/change_collision_sensitivity` | `dsr_msgs2/srv/ChangeCollisionSensitivity` | Reliable / Volatile / service default |
| Srv | `system/set_safety_mode` | `dsr_msgs2/srv/SetSafetyMode` | Reliable / Volatile / service default |
| Srv | `motion/move_joint` | `dsr_msgs2/srv/MoveJoint` | Reliable / Volatile / service default |
| Srv | `motion/move_line` | `dsr_msgs2/srv/MoveLine` | Reliable / Volatile / service default |
| Srv | `motion/move_jointx` | `dsr_msgs2/srv/MoveJointx` | Reliable / Volatile / service default |
| Srv | `motion/move_circle` | `dsr_msgs2/srv/MoveCircle` | Reliable / Volatile / service default |
| Srv | `motion/move_spline_joint` | `dsr_msgs2/srv/MoveSplineJoint` | Reliable / Volatile / service default |
| Srv | `motion/move_spline_task` | `dsr_msgs2/srv/MoveSplineTask` | Reliable / Volatile / service default |
| Srv | `motion/move_blending` | `dsr_msgs2/srv/MoveBlending` | Reliable / Volatile / service default |
| Srv | `motion/move_spiral` | `dsr_msgs2/srv/MoveSpiral` | Reliable / Volatile / service default |
| Srv | `motion/move_periodic` | `dsr_msgs2/srv/MovePeriodic` | Reliable / Volatile / service default |
| Srv | `motion/move_wait` | `dsr_msgs2/srv/MoveWait` | Reliable / Volatile / service default |
| Srv | `motion/jog` | `dsr_msgs2/srv/Jog` | Reliable / Volatile / service default |
| Srv | `motion/jog_multi` | `dsr_msgs2/srv/JogMulti` | Reliable / Volatile / service default |
| Srv | `motion/move_pause` | `dsr_msgs2/srv/MovePause` | Reliable / Volatile / service default |
| Srv | `motion/move_stop` | `dsr_msgs2/srv/MoveStop` | Reliable / Volatile / service default |
| Srv | `motion/move_resume` | `dsr_msgs2/srv/MoveResume` | Reliable / Volatile / service default |
| Srv | `motion/trans` | `dsr_msgs2/srv/Trans` | Reliable / Volatile / service default |
| Srv | `motion/fkin` | `dsr_msgs2/srv/Fkin` | Reliable / Volatile / service default |
| Srv | `motion/ikin` | `dsr_msgs2/srv/Ikin` | Reliable / Volatile / service default |
| Srv | `motion/set_ref_coord` | `dsr_msgs2/srv/SetRefCoord` | Reliable / Volatile / service default |
| Srv | `motion/move_home` | `dsr_msgs2/srv/MoveHome` | Reliable / Volatile / service default |
| Srv | `motion/check_motion` | `dsr_msgs2/srv/CheckMotion` | Reliable / Volatile / service default |
| Srv | `motion/change_operation_speed` | `dsr_msgs2/srv/ChangeOperationSpeed` | Reliable / Volatile / service default |
| Srv | `motion/enable_alter_motion` | `dsr_msgs2/srv/EnableAlterMotion` | Reliable / Volatile / service default |
| Srv | `motion/alter_motion` | `dsr_msgs2/srv/AlterMotion` | Reliable / Volatile / service default |
| Srv | `motion/disable_alter_motion` | `dsr_msgs2/srv/DisableAlterMotion` | Reliable / Volatile / service default |
| Srv | `motion/set_singularity_handling` | `dsr_msgs2/srv/SetSingularityHandling` | Reliable / Volatile / service default |
| Srv | `motion/set_singular_handling_force` | `dsr_msgs2/srv/SetSingularHandlingForce` | Reliable / Volatile / service default |
| Srv | `aux_control/get_control_mode` | `dsr_msgs2/srv/GetControlMode` | Reliable / Volatile / service default |
| Srv | `aux_control/get_control_space` | `dsr_msgs2/srv/GetControlSpace` | Reliable / Volatile / service default |
| Srv | `aux_control/get_current_posj` | `dsr_msgs2/srv/GetCurrentPosj` | Reliable / Volatile / service default |
| Srv | `aux_control/get_current_velj` | `dsr_msgs2/srv/GetCurrentVelj` | Reliable / Volatile / service default |
| Srv | `aux_control/get_desired_posj` | `dsr_msgs2/srv/GetDesiredPosj` | Reliable / Volatile / service default |
| Srv | `aux_control/get_desired_velj` | `dsr_msgs2/srv/GetDesiredVelj` | Reliable / Volatile / service default |
| Srv | `aux_control/get_current_posx` | `dsr_msgs2/srv/GetCurrentPosx` | Reliable / Volatile / service default |
| Srv | `aux_control/get_current_tool_flange_posx` | `dsr_msgs2/srv/GetCurrentToolFlangePosx` | Reliable / Volatile / service default |
| Srv | `aux_control/get_current_velx` | `dsr_msgs2/srv/GetCurrentVelx` | Reliable / Volatile / service default |
| Srv | `aux_control/get_desired_posx` | `dsr_msgs2/srv/GetDesiredPosx` | Reliable / Volatile / service default |
| Srv | `aux_control/get_desired_velx` | `dsr_msgs2/srv/GetDesiredVelx` | Reliable / Volatile / service default |
| Srv | `aux_control/get_current_solution_space` | `dsr_msgs2/srv/GetCurrentSolutionSpace` | Reliable / Volatile / service default |
| Srv | `aux_control/get_current_rotm` | `dsr_msgs2/srv/GetCurrentRotm` | Reliable / Volatile / service default |
| Srv | `aux_control/get_joint_torque` | `dsr_msgs2/srv/GetJointTorque` | Reliable / Volatile / service default |
| Srv | `aux_control/get_external_torque` | `dsr_msgs2/srv/GetExternalTorque` | Reliable / Volatile / service default |
| Srv | `aux_control/get_tool_force` | `dsr_msgs2/srv/GetToolForce` | Reliable / Volatile / service default |
| Srv | `aux_control/get_solution_space` | `dsr_msgs2/srv/GetSolutionSpace` | Reliable / Volatile / service default |
| Srv | `aux_control/get_orientation_error` | `dsr_msgs2/srv/GetOrientationError` | Reliable / Volatile / service default |
| Srv | `aux_control/get_robot_link_info` | `dsr_msgs2/srv/GetRobotLinkInfo` | Reliable / Volatile / service default |
| Srv | `force/parallel_axis1` | `dsr_msgs2/srv/ParallelAxis1` | Reliable / Volatile / service default |
| Srv | `force/parallel_axis2` | `dsr_msgs2/srv/ParallelAxis2` | Reliable / Volatile / service default |
| Srv | `force/align_axis1` | `dsr_msgs2/srv/AlignAxis1` | Reliable / Volatile / service default |
| Srv | `force/align_axis2` | `dsr_msgs2/srv/AlignAxis2` | Reliable / Volatile / service default |
| Srv | `force/is_done_bolt_tightening` | `dsr_msgs2/srv/IsDoneBoltTightening` | Reliable / Volatile / service default |
| Srv | `force/release_compliance_ctrl` | `dsr_msgs2/srv/ReleaseComplianceCtrl` | Reliable / Volatile / service default |
| Srv | `force/task_compliance_ctrl` | `dsr_msgs2/srv/TaskComplianceCtrl` | Reliable / Volatile / service default |
| Srv | `force/set_stiffnessx` | `dsr_msgs2/srv/SetStiffnessx` | Reliable / Volatile / service default |
| Srv | `force/calc_coord` | `dsr_msgs2/srv/CalcCoord` | Reliable / Volatile / service default |
| Srv | `force/set_user_cart_coord1` | `dsr_msgs2/srv/SetUserCartCoord1` | Reliable / Volatile / service default |
| Srv | `force/set_user_cart_coord2` | `dsr_msgs2/srv/SetUserCartCoord2` | Reliable / Volatile / service default |
| Srv | `force/set_user_cart_coord3` | `dsr_msgs2/srv/SetUserCartCoord3` | Reliable / Volatile / service default |
| Srv | `force/overwrite_user_cart_coord` | `dsr_msgs2/srv/OverwriteUserCartCoord` | Reliable / Volatile / service default |
| Srv | `force/get_user_cart_coord` | `dsr_msgs2/srv/GetUserCartCoord` | Reliable / Volatile / service default |
| Srv | `force/set_desired_force` | `dsr_msgs2/srv/SetDesiredForce` | Reliable / Volatile / service default |
| Srv | `force/release_force` | `dsr_msgs2/srv/ReleaseForce` | Reliable / Volatile / service default |
| Srv | `force/check_position_condition` | `dsr_msgs2/srv/CheckPositionCondition` | Reliable / Volatile / service default |
| Srv | `force/check_force_condition` | `dsr_msgs2/srv/CheckForceCondition` | Reliable / Volatile / service default |
| Srv | `force/check_orientation_condition1` | `dsr_msgs2/srv/CheckOrientationCondition1` | Reliable / Volatile / service default |
| Srv | `force/check_orientation_condition2` | `dsr_msgs2/srv/CheckOrientationCondition2` | Reliable / Volatile / service default |
| Srv | `force/coord_transform` | `dsr_msgs2/srv/CoordTransform` | Reliable / Volatile / service default |
| Srv | `force/get_workpiece_weight` | `dsr_msgs2/srv/GetWorkpieceWeight` | Reliable / Volatile / service default |
| Srv | `force/reset_workpiece_weight` | `dsr_msgs2/srv/ResetWorkpieceWeight` | Reliable / Volatile / service default |
| Srv | `io/set_ctrl_box_digital_output` | `dsr_msgs2/srv/SetCtrlBoxDigitalOutput` | Reliable / Volatile / service default |
| Srv | `io/get_ctrl_box_digital_input` | `dsr_msgs2/srv/GetCtrlBoxDigitalInput` | Reliable / Volatile / service default |
| Srv | `io/set_tool_digital_output` | `dsr_msgs2/srv/SetToolDigitalOutput` | Reliable / Volatile / service default |
| Srv | `io/get_tool_digital_input` | `dsr_msgs2/srv/GetToolDigitalInput` | Reliable / Volatile / service default |
| Srv | `io/set_ctrl_box_analog_output` | `dsr_msgs2/srv/SetCtrlBoxAnalogOutput` | Reliable / Volatile / service default |
| Srv | `io/get_ctrl_box_analog_input` | `dsr_msgs2/srv/GetCtrlBoxAnalogInput` | Reliable / Volatile / service default |
| Srv | `io/set_ctrl_box_analog_output_type` | `dsr_msgs2/srv/SetCtrlBoxAnalogOutputType` | Reliable / Volatile / service default |
| Srv | `io/set_ctrl_box_analog_input_type` | `dsr_msgs2/srv/SetCtrlBoxAnalogInputType` | Reliable / Volatile / service default |
| Srv | `io/get_ctrl_box_digital_output` | `dsr_msgs2/srv/GetCtrlBoxDigitalOutput` | Reliable / Volatile / service default |
| Srv | `io/get_tool_digital_output` | `dsr_msgs2/srv/GetToolDigitalOutput` | Reliable / Volatile / service default |
| Srv | `modbus/set_modbus_output` | `dsr_msgs2/srv/SetModbusOutput` | Reliable / Volatile / service default |
| Srv | `modbus/get_modbus_input` | `dsr_msgs2/srv/GetModbusInput` | Reliable / Volatile / service default |
| Srv | `modbus/config_create_modbus` | `dsr_msgs2/srv/ConfigCreateModbus` | Reliable / Volatile / service default |
| Srv | `modbus/config_delete_modbus` | `dsr_msgs2/srv/ConfigDeleteModbus` | Reliable / Volatile / service default |
| Srv | `tcp/config_create_tcp` | `dsr_msgs2/srv/ConfigCreateTcp` | Reliable / Volatile / service default |
| Srv | `tcp/config_delete_tcp` | `dsr_msgs2/srv/ConfigDeleteTcp` | Reliable / Volatile / service default |
| Srv | `tcp/get_current_tcp` | `dsr_msgs2/srv/GetCurrentTcp` | Reliable / Volatile / service default |
| Srv | `tcp/set_current_tcp` | `dsr_msgs2/srv/SetCurrentTcp` | Reliable / Volatile / service default |
| Srv | `tool/config_create_tool` | `dsr_msgs2/srv/ConfigCreateTool` | Reliable / Volatile / service default |
| Srv | `tool/config_delete_tool` | `dsr_msgs2/srv/ConfigDeleteTool` | Reliable / Volatile / service default |
| Srv | `tool/get_current_tool` | `dsr_msgs2/srv/GetCurrentTool` | Reliable / Volatile / service default |
| Srv | `tool/set_current_tool` | `dsr_msgs2/srv/SetCurrentTool` | Reliable / Volatile / service default |
| Srv | `tool/set_tool_shape` | `dsr_msgs2/srv/SetToolShape` | Reliable / Volatile / service default |
| Srv | `drl/drl_pause` | `dsr_msgs2/srv/DrlPause` | Reliable / Volatile / service default |
| Srv | `drl/drl_start` | `dsr_msgs2/srv/DrlStart` | Reliable / Volatile / service default |
| Srv | `drl/drl_stop` | `dsr_msgs2/srv/DrlStop` | Reliable / Volatile / service default |
| Srv | `drl/drl_resume` | `dsr_msgs2/srv/DrlResume` | Reliable / Volatile / service default |
| Srv | `drl/get_drl_state` | `dsr_msgs2/srv/GetDrlState` | Reliable / Volatile / service default |
| Srv | `realtime/connect_rt_control` | `dsr_msgs2/srv/ConnectRtControl` | Reliable / Volatile / service default |
| Srv | `realtime/disconnect_rt_control` | `dsr_msgs2/srv/DisconnectRtControl` | Reliable / Volatile / service default |
| Srv | `realtime/get_rt_control_output_version_list` | `dsr_msgs2/srv/GetRtControlOutputVersionList` | Reliable / Volatile / service default |
| Srv | `realtime/get_rt_control_input_version_list` | `dsr_msgs2/srv/GetRtControlInputVersionList` | Reliable / Volatile / service default |
| Srv | `realtime/get_rt_control_input_data_list` | `dsr_msgs2/srv/GetRtControlInputDataList` | Reliable / Volatile / service default |
| Srv | `realtime/get_rt_control_output_data_list` | `dsr_msgs2/srv/GetRtControlOutputDataList` | Reliable / Volatile / service default |
| Srv | `realtime/set_rt_control_input` | `dsr_msgs2/srv/SetRtControlInput` | Reliable / Volatile / service default |
| Srv | `realtime/set_rt_control_output` | `dsr_msgs2/srv/SetRtControlOutput` | Reliable / Volatile / service default |
| Srv | `realtime/start_rt_control` | `dsr_msgs2/srv/StartRtControl` | Reliable / Volatile / service default |
| Srv | `realtime/stop_rt_control` | `dsr_msgs2/srv/StopRtControl` | Reliable / Volatile / service default |
| Srv | `realtime/set_velj_rt` | `dsr_msgs2/srv/SetVeljRt` | Reliable / Volatile / service default |
| Srv | `realtime/set_accj_rt` | `dsr_msgs2/srv/SetAccjRt` | Reliable / Volatile / service default |
| Srv | `realtime/set_velx_rt` | `dsr_msgs2/srv/SetVelxRt` | Reliable / Volatile / service default |
| Srv | `realtime/set_accx_rt` | `dsr_msgs2/srv/SetAccxRt` | Reliable / Volatile / service default |
| Srv | `realtime/read_data_rt` | `dsr_msgs2/srv/ReadDataRt` | Reliable / Volatile / service default |
| Srv | `realtime/write_data_rt` | `dsr_msgs2/srv/WriteDataRt` | Reliable / Volatile / service default |
| Srv | `plc/get_input_register_int` | `dsr_msgs2/srv/GetInputRegisterInt` | Reliable / Volatile / service default |
| Srv | `plc/get_input_register_bit` | `dsr_msgs2/srv/GetInputRegisterBit` | Reliable / Volatile / service default |
| Srv | `plc/get_input_register_float` | `dsr_msgs2/srv/GetInputRegisterFloat` | Reliable / Volatile / service default |
| Srv | `plc/set_output_register_int` | `dsr_msgs2/srv/SetOutputRegisterInt` | Reliable / Volatile / service default |
| Srv | `plc/set_output_register_bit` | `dsr_msgs2/srv/SetOutputRegisterBit` | Reliable / Volatile / service default |
| Srv | `plc/set_output_register_float` | `dsr_msgs2/srv/SetOutputRegisterFloat` | Reliable / Volatile / service default |
| Srv | `plc/get_output_register_int` | `dsr_msgs2/srv/GetOutputRegisterInt` | Reliable / Volatile / service default |
| Srv | `plc/get_output_register_bit` | `dsr_msgs2/srv/GetOutputRegisterBit` | Reliable / Volatile / service default |
| Srv | `plc/get_output_register_float` | `dsr_msgs2/srv/GetOutputRegisterFloat` | Reliable / Volatile / service default |
| Topic Sub | `alter_motion_stream` | `dsr_msgs2/msg/AlterMotionStream` | Reliable / Volatile / depth 20 |
| Topic Sub | `servoj_stream` | `dsr_msgs2/msg/ServojStream` | Reliable / Volatile / depth 20 |
| Topic Sub | `servol_stream` | `dsr_msgs2/msg/ServolStream` | Reliable / Volatile / depth 20 |
| Topic Sub | `speedj_stream` | `dsr_msgs2/msg/SpeedjStream` | Reliable / Volatile / depth 20 |
| Topic Sub | `speedl_stream` | `dsr_msgs2/msg/SpeedlStream` | Reliable / Volatile / depth 10 |
| Topic Sub | `servoj_rt_stream` | `dsr_msgs2/msg/ServojRtStream` | Reliable / Volatile / depth 20 |
| Topic Sub | `servol_rt_stream` | `dsr_msgs2/msg/ServolRtStream` | Reliable / Volatile / depth 20 |
| Topic Sub | `speedj_rt_stream` | `dsr_msgs2/msg/SpeedjRtStream` | Reliable / Volatile / depth 20 |
| Topic Sub | `speedl_rt_stream` | `dsr_msgs2/msg/SpeedlRtStream` | Reliable / Volatile / depth 20 |
| Topic Sub | `torque_rt_stream` | `dsr_msgs2/msg/TorqueRtStream` | Reliable / Volatile / depth 20 |
| Topic Pub | `error` | `dsr_msgs2/msg/RobotError` | Reliable / Volatile / depth 100 |
| Topic Pub | `robot_disconnection` | `dsr_msgs2/msg/RobotDisconnection` | Reliable / Volatile / depth 100 |
| Action | `motion/movej_h2r` | `dsr_msgs2/action/MovejH2r` | ROS 2 action 기본 QoS |
| Action | `motion/movel_h2r` | `dsr_msgs2/action/MovelH2r` | ROS 2 action 기본 QoS |

단위 원칙: `pos`/`fpos` joint 배열은 Doosan service 기준 degree, task pose 배열은 `[x,y,z,rx,ry,rz]`이며 선형 위치 mm, 회전 deg이다. `vel`/`acc` joint 값은 deg/s 및 deg/s^2, task 선형 값은 mm/s 및 mm/s^2, task 회전 값은 deg/s 및 deg/s^2이다. `time`은 sec, force 계열 `fd`는 N 또는 Nm 축 성분, stiffness 계열은 Doosan API의 N/m 및 Nm/rad 계열 값이다. 표준 ROS `JointTrajectory`/`JointState`/MoveIt controller 인터페이스는 rad, rad/s, N·m을 사용한다.

### 11.2 Service 원문 필드

#### `system/set_robot_mode` (`dsr_msgs2/srv/SetRobotMode`)

원문: `dsr_msgs2/srv/system/SetRobotMode.srv`

```srv
#____________________________________________________________________________________________
# set_robot_mode
# Change the robot-mode
# 0 : ROBOT_MODE_MANUAL
# 1 : ROBOT_MODE_AUTONOMOUS
# 2 :ROBOT_MODE_MEASURE
# drfl.SetRobotMode()
#____________________________________________________________________________________________

int8 robot_mode # <Robot_Mode>
---
bool success
```

#### `system/get_robot_mode` (`dsr_msgs2/srv/GetRobotMode`)

원문: `dsr_msgs2/srv/system/GetRobotMode.srv`

```srv
#____________________________________________________________________________________________
# get_robot_mode
# Return to current robot-mode
# 0 : ROBOT_MODE_MANUAL
# 1 : ROBOT_MODE_AUTONOMOUS
# 2 : ROBOT_MODE_MEASURE
# drfl.GetRobotMode()
#____________________________________________________________________________________________

---
int8 robot_mode
bool        success
```

#### `system/set_robot_system` (`dsr_msgs2/srv/SetRobotSystem`)

원문: `dsr_msgs2/srv/system/SetRobotSystem.srv`

```srv
#____________________________________________________________________________________________
# set_robot_system
#____________________________________________________________________________________________

int8 robot_system   # 0 : ROBOT_SYSTEM_REAL, 1 : ROBOT_SYSTEM_VIRTUAL
---
bool success
```

#### `system/get_robot_system` (`dsr_msgs2/srv/GetRobotSystem`)

원문: `dsr_msgs2/srv/system/GetRobotSystem.srv`

```srv
#____________________________________________________________________________________________
# get_robot_system
#____________________________________________________________________________________________

---
int8 robot_system   # 0 : ROBOT_SYSTEM_REAL
                    # 1 : ROBOT_SYSTEM_VIRTUAL
bool        success
```

#### `system/get_robot_state` (`dsr_msgs2/srv/GetRobotState`)

원문: `dsr_msgs2/srv/system/GetRobotState.srv`

```srv
#____________________________________________________________________________________________
# get_robot_state
#____________________________________________________________________________________________

---
int8 robot_state    # 0 : STATE_INITIALIZING
                    # 1 : STATE_STANDBY
                    # 2 : STATE_MOVING
                    # 3 : STATE_SAFE_OFF
                    # 4 : STATE_TEACHING
                    # 5 : STATE_SAFE_STOP
                    # 6 : STATE_EMERGENCY_STOP:
                    # 7 : STATE_HOMMING
                    # 8 : STATE_RECOVERY
                    # 9 : eSTATE_SAFE_STOP2
                    # 10: STATE_SAFE_OFF2
                    # 11: STATE_RESERVED1
                    # 12: STATE_RESERVED2
                    # 13: STATE_RESERVED3
                    # 14: STATE_RESERVED4
                    # 15: STATE_NOT_READY
bool        success
```

#### `system/set_robot_speed_mode` (`dsr_msgs2/srv/SetRobotSpeedMode`)

원문: `dsr_msgs2/srv/system/SetRobotSpeedMode.srv`

```srv
#____________________________________________________________________________________________
# set_robot_speed_mode
#____________________________________________________________________________________________

int8 speed_mode # 0 : SPEED_NORMAL_MODE, 1 : SPEED_REDUCED_MODE
---
bool success
```

#### `system/get_robot_speed_mode` (`dsr_msgs2/srv/GetRobotSpeedMode`)

원문: `dsr_msgs2/srv/system/GetRobotSpeedMode.srv`

```srv
#____________________________________________________________________________________________
# get_robot_speed_mode
#____________________________________________________________________________________________

---
int8 speed_mode # 0 : SPEED_NORMAL_MODE
                # 1 : SPEED_REDUCED_MODE
bool        success
```

#### `system/get_current_pose` (`dsr_msgs2/srv/GetCurrentPose`)

원문: `dsr_msgs2/srv/system/GetCurrentPose.srv`

```srv
#____________________________________________________________________________________________
# get_current_pose
#____________________________________________________________________________________________

int8 space_type # 0=ROBOT_SPACE_JOINT, 1=ROBOT_SPACE_TASK
---
float64[6] pos
bool       success
```

#### `system/set_safe_stop_reset_type` (`dsr_msgs2/srv/SetSafeStopResetType`)

원문: `dsr_msgs2/srv/system/SetSafeStopResetType.srv`

```srv
#____________________________________________________________________________________________
# set_safe_stop_reset_type
#____________________________________________________________________________________________

int8 reset_type     # 0=SAFE_STOP_RESET_TYPE_DEFAULT = SAFE_STOP_RESET_TYPE_PROGRAM_STOP , 1= SAFE_STOP_RESET_TYPE_PROGRAM_RESUME 
---
bool success
```

#### `system/get_last_alarm` (`dsr_msgs2/srv/GetLastAlarm`)

원문: `dsr_msgs2/srv/system/GetLastAlarm.srv`

```srv
#____________________________________________________________________________________________
# get_last_alarm
###Struct.LOG_ARARM
#____________________________________________________________________________________________

---
LogAlarm log_alarm
bool        success
```

#### `system/servo_off` (`dsr_msgs2/srv/ServoOff`)

원문: `dsr_msgs2/srv/system/ServoOff.srv`

```srv
#____________________________________________________________________________________________
# servo off
# STOP_TYPE_QUICK_STO = 0,
# STOP_TYPE_QUICK,
# STOP_TYPE_SLOW,
# STOP_TYPE_HOLD,
# STOP_TYPE_EMERGENCY = STOP_TYPE_HOLD,
#____________________________________________________________________________________________

int8 STOP_TYPE_QUICK_STO = 0
int8 STOP_TYPE_QUICK = 1
int8 STOP_TYPE_SLOW = 2
int8 STOP_TYPE_HOLD = 3
int8 STOP_TYPE_EMERGENCY = 3

int8 stop_type     
---
bool success
```

#### `system/set_robot_control` (`dsr_msgs2/srv/SetRobotControl`)

원문: `dsr_msgs2/srv/system/SetRobotControl.srv`

```srv
#____________________________________________________________________________________________
# set_robot_control
# 0 : CONTROL_INIT_CONFIG
# executes the function to convert from STATE_NOT_READY to STATE_INITIALIZING, and only the T/P applicatiexecutes this function.
# 1 : CONTROL_ENABLE_OPERATION
# executes the function to convert from STATE_INITIALIZING to STATE_STANDBY, and only the T/P applicatiexecutes this function.
# 2 : CONTROL_RESET_SAFET_STOP
# executes the function to convert from STATE_SAFE_STOP to STATE_STANDBY. Program restart can be set in the case of automatic mode.
# 3 : CONTROL_RESET_SAFET_OFF
# executes the function to convert from STATE_SAFE_OFF to STATE_STANDBY.
# 4 : CONTROL_RECOVERY_SAFE_STOP
# executes the S/W-based function to convert from STATE_SAFE_STOP2 to STATE_RECOVERY.
# 5 : CONTROL_RECOVERY_SAFE_OFF
# executes the S/W-based function to convert from STATE_SAFE_OFF2 to STATE_RECOVERY.
# 6 : CONTROL_RECOVERY_BACKDRIVE
# executes the H/W-based function to convert from STATE_SAFE_OFF2 to STATE_RECOVERY. cannot be converted into STATE_STANDBY, and robot controller power should be rebooted.
# 7 : CONTROL_RESET_RECOVERY
# executes the function to convert from STATE_RECOVERY to STATE_STANDBY.
#____________________________________________________________________________________________

int8 robot_control 
---
bool success
```

#### `system/change_collision_sensitivity` (`dsr_msgs2/srv/ChangeCollisionSensitivity`)

원문: `dsr_msgs2/srv/system/ChangeCollisionSensitivity.srv`

```srv
#____________________________________________________________________________________________
# change_collision_sensitivity
#____________________________________________________________________________________________

int8 sensitivity   # 0 ~ 100 
---
bool success
```

#### `system/set_safety_mode` (`dsr_msgs2/srv/SetSafetyMode`)

원문: `dsr_msgs2/srv/system/SetSafetyMode.srv`

```srv
#____________________________________________________________________________________________
# set_safety_mode
# safety_mode:
# 0: SAFETY_MODE_MANUAL
# 1: SAFETY_MODE_AUTONOMOUS
# 2: SAFETY_MODE_RECOVERY
# 3: SAFETY_MODE_BACKDRIVE
# 4: SAFETY_MODE_MEASURE
# 5: SAFETY_MODE_INITIALIZE
# safety_event:
# 0: SAFETY_MODE_EVENT_ENTER
# 1: SAFETY_MODE_EVENT_MOVE
# 2: SAFETY_MODE_EVENT_STOP
# 3: SAFETY_MODE_EVENT_LAST
#____________________________________________________________________________________________

int8 safety_mode
int8 safety_event
---
bool success
```

#### `motion/move_joint` (`dsr_msgs2/srv/MoveJoint`)

원문: `dsr_msgs2/srv/motion/MoveJoint.srv`

```srv
#____________________________________________________________________________________________
# move_joint  
# The robot moves to the target joint position (pos) from the current joint position.
#____________________________________________________________________________________________

float64[6] pos               # target joint angle list [degree] 
float64    vel               # set velocity: [deg/sec]
float64    acc               # set acceleration: [deg/sec2]
float64    time #= 0.0       # Time [sec] 
float64    radius #=0.0      # Radius under blending mode [mm] 
int8       mode #= 0         # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8       blend_type #= 0    # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8       sync_type #=0      # SYNC = 0, ASYNC = 1
---
bool success
```

#### `motion/move_line` (`dsr_msgs2/srv/MoveLine`)

원문: `dsr_msgs2/srv/motion/MoveLine.srv`

```srv
#____________________________________________________________________________________________
# move_line  
#____________________________________________________________________________________________

float64[6] pos               # target  
float64[2] vel               # set velocity: [mm/sec], [deg/sec]
float64[2] acc               # set acceleration: [mm/sec2], [deg/sec2]
float64    time #= 0.0       # Time [sec] 
float64    radius #=0.0      # Radius under blending mode [mm] 
int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0         # DR_MV_MOD_ABS(0), DR_MV_MOD_REL(1) 
int8       blend_type #= 0    # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8       sync_type #=0      # SYNC = 0, ASYNC = 1
---
bool success
```

#### `motion/move_jointx` (`dsr_msgs2/srv/MoveJointx`)

원문: `dsr_msgs2/srv/motion/MoveJointx.srv`

```srv
#____________________________________________________________________________________________
# move_jointx  
#____________________________________________________________________________________________

float64[6] pos              # target  
float64    vel              # set velocity: [deg/sec]
float64    acc              # set acceleration: [deg/sec2] 
float64    time #= 0.0      # Time [sec] 
float64    radius #=0.0     # Radius under blending mode [mm]   
int8       ref              # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                            # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0        # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8       blend_type #= 0   # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8       sol              # SolutionSpace : 0~7
int8       sync_type #=0     # SYNC = 0, ASYNC = 1
---
bool success
```

#### `motion/move_circle` (`dsr_msgs2/srv/MoveCircle`)

원문: `dsr_msgs2/srv/motion/MoveCircle.srv`

```srv
#____________________________________________________________________________________________
# move_circle  
#____________________________________________________________________________________________

std_msgs/Float64MultiArray[] pos  # target[2][6]  
float64[2]      vel               # set velocity: [mm/sec], [deg/sec]
float64[2]      acc               # set acceleration: [mm/sec2], [deg/sec2]
float64         time #= 0.0       # Time [sec] 
float64         radius #=0.0      # Radius under blending mode [mm] 
int8            ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                                  # <DR_WORLD is only available in M2.40 or later> 
int8            mode #= 0         # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
float64         angle1 #= 0.0     # angle1 [degree]
float64         angle2 #= 0.0     # angle2 [degree]
int8            blend_type #= 0    # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8            sync_type #=0      # SYNC = 0, ASYNC = 1
---
bool success
```

#### `motion/move_spline_joint` (`dsr_msgs2/srv/MoveSplineJoint`)

원문: `dsr_msgs2/srv/motion/MoveSplineJoint.srv`

```srv
#____________________________________________________________________________________________
# move_spline_joint  
###float64[100][6] pos         # target
#____________________________________________________________________________________________

std_msgs/Float64MultiArray[] pos         # target [100][6] pos
int8       pos_cnt                       # target cnt 
float64[6]    vel                        # set joint velocity: [deg/sec]
float64[6]    acc                        # set joint acceleration: [deg/sec2] 
float64    time #= 0.0                   # Time [sec] 
int8       mode #= 0                     # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8       sync_type #=0                 # SYNC = 0, ASYNC = 1
---
bool success
```

#### `motion/move_spline_task` (`dsr_msgs2/srv/MoveSplineTask`)

원문: `dsr_msgs2/srv/motion/MoveSplineTask.srv`

```srv
#____________________________________________________________________________________________
# move_spline_task  
###float64[100][6] pos            # target
#____________________________________________________________________________________________

std_msgs/Float64MultiArray[] pos  # target 
int8            pos_cnt            # target cnt 
float64[2]      vel               # set velocity: [mm/sec], [deg/sec]
float64[2]      acc               # set acceleration: [mm/sec2], [deg/sec2]
float64         time #= 0.0       # Time [sec] 
int8            ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                                  # <DR_WORLD is only available in M2.40 or later 
int8            mode #= 0         # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8            opt  #= 0         # SPLINE_VELOCITY_OPTION_DEFAULT=0, SPLINE_VELOCITY_OPTION_CONST=1 
int8            sync_type #=0      # SYNC = 0, ASYNC = 1
---
bool success
```

#### `motion/move_blending` (`dsr_msgs2/srv/MoveBlending`)

원문: `dsr_msgs2/srv/motion/MoveBlending.srv`

```srv
#____________________________________________________________________________________________
# move_blending  
#float64[50][6] pos              # target
#std_msgs/Float64MultiArray[] pos1   # target1 [50][6]
#std_msgs/Float64MultiArray[] pos2   # target2 [50][6]
#int8[50]       segment              # LINE=0 , CIRCLE=1
#float64[50]    radius               # Radius of segment 
#____________________________________________________________________________________________

std_msgs/Float64MultiArray[] segment #50 x (pos1[6]:pos2[6]:type[1]:radius[1])        
int8           pos_cnt               # target cnt 
float64[2]     vel                  # set velocity: [mm/sec], [deg/sec]
float64[2]     acc                  # set acceleration: [mm/sec2], [deg/sec2]
float64        time #= 0.0          # Time [sec] 
int8           ref                  # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                                    # <DR_WORLD is only available in M2.40 or later 
int8           mode #= 0            # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8           sync_type #=0         # SYNC = 0, ASYNC = 1
---
bool success
```

#### `motion/move_spiral` (`dsr_msgs2/srv/MoveSpiral`)

원문: `dsr_msgs2/srv/motion/MoveSpiral.srv`

```srv
#____________________________________________________________________________________________
# move_spiral  
#____________________________________________________________________________________________

float64    revolution       # Total number of revolutions 
float64    max_radius       # Final spiral radius [mm]
float64    max_length       # Distance moved in the axis direction [mm]
float64[3] target_pos       # Target position [mm]. If used, max_radius and max_length are ignored
float64[2] vel              # set velocity: [mm/sec], [deg/sec]
float64[2] acc              # set acceleration: [mm/sec2], [deg/sec2]
float64    time #= 0.0      # Total execution time <sec> 
int8       task_axis        # TASK_AXIS_X = 0, TASK_AXIS_Y = 1, TASK_AXIS_Z = 2   
int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                            # <DR_WORLD is only available in M2.40 or later 
int8       mode #= 0        # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8       spiral_dir #= 0  # MOVE_SPIRAL_OUTWARD=0, MOVE_SPIRAL_INWARD=1
int8       rot_dir #=0      # MOVE_SPIRAL_FORWARD=0, MOVE_SPIRAL_REVERSE=1 
int8       sync_type #=0    # SYNC = 0, ASYNC = 1 
---
bool success
```

#### `motion/move_periodic` (`dsr_msgs2/srv/MovePeriodic`)

원문: `dsr_msgs2/srv/motion/MovePeriodic.srv`

```srv
#____________________________________________________________________________________________
# move_periodic  
#____________________________________________________________________________________________

float64[6] amp              # Amplitude (motion between -amp and +amp) [mm] or [deg]   
float64[6] periodic         # Period (time for 1 cycle) [sec]
float64    acc              # Acc-, dec- time [sec] 
int8       repeat           # Repetition count 
int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                            # <DR_WORLD is only available in M2.40 or later 

int8       sync_type #=0     # SYNC = 0, ASYNC = 1
---
bool success
```

#### `motion/move_wait` (`dsr_msgs2/srv/MoveWait`)

원문: `dsr_msgs2/srv/motion/MoveWait.srv`

```srv
#____________________________________________________________________________________________
# move_wait
# This Service sets the waiting time between the previous motion command 
# and the motion command in the next line.
#____________________________________________________________________________________________

---
bool success
```

#### `motion/jog` (`dsr_msgs2/srv/Jog`)

원문: `dsr_msgs2/srv/motion/Jog.srv`

```srv
#____________________________________________________________________________________________
# single jog
# single jog speed = (250mm/s) x speed [%] 
#____________________________________________________________________________________________

int8 jog_axis          # 0 ~ 5 : JOINT 1 ~ 6 
                       # 6 ~ 11: TASK 1 ~ 6 (X,Y,Z,rx,ry,rz)
int8 move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL
float64 speed          # jog speed [%] : + forward , 0=stop, - backward  
---
bool success
```

#### `motion/jog_multi` (`dsr_msgs2/srv/JogMulti`)

원문: `dsr_msgs2/srv/motion/JogMulti.srv`

```srv
#____________________________________________________________________________________________
# multi jog speed = (250mm/s x 1.73) x unit vecter x speed [%] 
#____________________________________________________________________________________________

float64[6] jog_axis    # unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~ +1.0 
int8 move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD
float64 speed          # jog speed [%]  
---
bool success
```

#### `motion/move_pause` (`dsr_msgs2/srv/MovePause`)

원문: `dsr_msgs2/srv/motion/MovePause.srv`

```srv
#____________________________________________________________________________________________
# motion pause
#____________________________________________________________________________________________

---
bool success
```

#### `motion/move_stop` (`dsr_msgs2/srv/MoveStop`)

원문: `dsr_msgs2/srv/motion/MoveStop.srv`

```srv
#____________________________________________________________________________________________
# stop()
# 인자 설명 추가 필요!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#____________________________________________________________________________________________

int32 stop_mode         # DR_QSTOP_STO(0) : Quick stop (Stop Category 1 without STO(Safe Torque Off)
                        # DR_QSTOP(1)     : Quick stop (Stop Category 2)
                        # DR_SSTO(2)      : Soft Stop
                        # DR_HOLD(3)      : HOLD stop
---
bool success
```

#### `motion/move_resume` (`dsr_msgs2/srv/MoveResume`)

원문: `dsr_msgs2/srv/motion/MoveResume.srv`

```srv
#____________________________________________________________________________________________
# motion pause
#____________________________________________________________________________________________

---
bool success
```

#### `motion/trans` (`dsr_msgs2/srv/Trans`)

원문: `dsr_msgs2/srv/motion/Trans.srv`

```srv
#____________________________________________________________________________________________
# trans  
#____________________________________________________________________________________________

float64[6] pos               # task pos(posx)  
float64[6] delta             # delta (posx)  
int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       ref_out #= 0      # DR_BASE(0), DR_WORLD(2)
                             # <ref_out is only available in M2.40 or later>
---
float64[6] trans_pos         # trans pos(posx) 
bool       success
```

#### `motion/fkin` (`dsr_msgs2/srv/Fkin`)

원문: `dsr_msgs2/srv/motion/Fkin.srv`

```srv
#____________________________________________________________________________________________
# fkin  
#____________________________________________________________________________________________

float64[6] pos               # joint pos(posj)  
int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
                             # <ref is only available in M2.40 or later> 
---
float64[6] conv_posx         # task pos(posx)
bool       success
```

#### `motion/ikin` (`dsr_msgs2/srv/Ikin`)

원문: `dsr_msgs2/srv/motion/Ikin.srv`

```srv
#____________________________________________________________________________________________
# ikin  
#____________________________________________________________________________________________

float64[6] pos               # task pos(posx)  
int8       sol_space         # solution space : 0 ~ 7
int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
                             # <ref is only available in M2.40 or later> 
---
float64[6] conv_posj         # joint pos(posj)  
bool       success
```

#### `motion/set_ref_coord` (`dsr_msgs2/srv/SetRefCoord`)

원문: `dsr_msgs2/srv/motion/SetRefCoord.srv`

```srv
#____________________________________________________________________________________________
# set_ref_coord 
#____________________________________________________________________________________________

int8       coord            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200)
                            # <DR_WORLD is only available in M2.40 or later> 
---
bool success
```

#### `motion/move_home` (`dsr_msgs2/srv/MoveHome`)

원문: `dsr_msgs2/srv/motion/MoveHome.srv`

```srv
#____________________________________________________________________________________________
# move_home
# Homing is performed by moving to the joint motion to the mechanical or user defined home position.
# According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.
#____________________________________________________________________________________________

int8       target           # DR_HOME_TARGET_MECHANIC(0) : Mechanical home, joint angle (0,0,0,0,0,0)
                            # DR_HOME_TARGET_USER(1)     : user home
---
int8       res              # 0=success, otherwise fail 
bool       success
```

#### `motion/check_motion` (`dsr_msgs2/srv/CheckMotion`)

원문: `dsr_msgs2/srv/motion/CheckMotion.srv`

```srv
#____________________________________________________________________________________________
# check_motion
# return status of the currently active motion.
# Homing is performed by moving to the joint motion to the mechanical or user defined home position.
# According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.
#____________________________________________________________________________________________

---
int8       status          # DR_STATE_IDLE(0) : no motion in action
                           # DR_STATE_INIT(1) : motion being calculated
                           # DR_STATE_BUSY(2) : motion in operation
bool       success
```

#### `motion/change_operation_speed` (`dsr_msgs2/srv/ChangeOperationSpeed`)

원문: `dsr_msgs2/srv/motion/ChangeOperationSpeed.srv`

```srv
#____________________________________________________________________________________________
# change_operation_speed
#____________________________________________________________________________________________

int8 speed              # operation speed: (1~100)
---
bool success
```

#### `motion/enable_alter_motion` (`dsr_msgs2/srv/EnableAlterMotion`)

원문: `dsr_msgs2/srv/motion/EnableAlterMotion.srv`

```srv
#____________________________________________________________________________________________
# enable_alter_motion  
# 
#____________________________________________________________________________________________

int32      n                 # Cycle time number 
int8       mode              # DR_DPOS(0) : accumulation amount, DR_DVEL(1) : increment amount 
int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200) 
                             # <ref is only available in M2.40 or later> 
float64[2] limit_dpos        # First value : limitation of position[mm], Second value : limitation of orientation[deg]
float64[2] limit_dpos_per    # First value : limitation of position[mm], Second value : limitation of orientation[deg]

---
bool success
```

#### `motion/alter_motion` (`dsr_msgs2/srv/AlterMotion`)

원문: `dsr_msgs2/srv/motion/AlterMotion.srv`

```srv
#____________________________________________________________________________________________
# alter_motion  
# 
#____________________________________________________________________________________________

float64[6] pos               # position  
---
bool success
```

#### `motion/disable_alter_motion` (`dsr_msgs2/srv/DisableAlterMotion`)

원문: `dsr_msgs2/srv/motion/DisableAlterMotion.srv`

```srv
#____________________________________________________________________________________________
# disable_alter_motion  
# deactivates alter motion
#____________________________________________________________________________________________

---
bool success
```

#### `motion/set_singularity_handling` (`dsr_msgs2/srv/SetSingularityHandling`)

원문: `dsr_msgs2/srv/motion/SetSingularityHandling.srv`

```srv
#____________________________________________________________________________________________
# set_singularity_handling
# 
#____________________________________________________________________________________________

int8       mode         # DR_AVOID(0)     : Automatic avoidance mode
                        # DR_TASK_STOP(1) : Deceleration/ Warning/ Task termination
                        # DR_VAR_VEL(2)   : Variable velocity mode

---
bool success
```

#### `motion/set_singular_handling_force` (`dsr_msgs2/srv/SetSingularHandlingForce`)

원문: `dsr_msgs2/srv/motion/SetSingularHandlingForce.srv`

```srv
#____________________________________________________________________________________________
# set_singular_handling_force
# 
#____________________________________________________________________________________________

int8 mode     # DR_SINGULARITY_ERROR(0)  : Return error when force control/compliance control is used
              #                            within singularity area
              # DR_SINGULARITY_IGNORE(1) : Ignore error processing
---
bool success
```

#### `aux_control/get_control_mode` (`dsr_msgs2/srv/GetControlMode`)

원문: `dsr_msgs2/srv/aux_control/GetControlMode.srv`

```srv
#____________________________________________________________________________________________
# get_control_mode()  
#____________________________________________________________________________________________
# This service returns the current control mode.

---
int8    control_mode        # Control mode : Position control mode(3), Torque control mode(4)
bool    success
```

#### `aux_control/get_control_space` (`dsr_msgs2/srv/GetControlSpace`)

원문: `dsr_msgs2/srv/aux_control/GetControlSpace.srv`

```srv
#____________________________________________________________________________________________
# get_control_space()  
#____________________________________________________________________________________________
# This service returns the current control space.

---
int8    space        # Control mode : Joint space control(1), Task space control(2)
bool    success
```

#### `aux_control/get_current_posj` (`dsr_msgs2/srv/GetCurrentPosj`)

원문: `dsr_msgs2/srv/aux_control/GetCurrentPosj.srv`

```srv
#____________________________________________________________________________________________
# get_current_posj()  
#____________________________________________________________________________________________
# This service returns the current joint angle.

---
float64[6] pos               # joint pos(posj)  
bool       success
```

#### `aux_control/get_current_velj` (`dsr_msgs2/srv/GetCurrentVelj`)

원문: `dsr_msgs2/srv/aux_control/GetCurrentVelj.srv`

```srv
#____________________________________________________________________________________________
# get_current_velj()  
#____________________________________________________________________________________________
# This service returns the current target joint velocity. It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic command.

---
float64[6]  joint_speed               # joint speed 
bool        success
```

#### `aux_control/get_desired_posj` (`dsr_msgs2/srv/GetDesiredPosj`)

원문: `dsr_msgs2/srv/aux_control/GetDesiredPosj.srv`

```srv
#____________________________________________________________________________________________
# get_desired_posj()  
#____________________________________________________________________________________________
# This service returns the current target joint angle.
# It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic service.

---
float64[6] pos               # joint pos(posj)  
bool       success
```

#### `aux_control/get_desired_velj` (`dsr_msgs2/srv/GetDesiredVelj`)

원문: `dsr_msgs2/srv/aux_control/GetDesiredVelj.srv`

```srv
#____________________________________________________________________________________________
# get_desired_velj()  
#____________________________________________________________________________________________

---
float64[6] joint_vel               # Target joint velocity 
bool       success
```

#### `aux_control/get_current_posx` (`dsr_msgs2/srv/GetCurrentPosx`)

원문: `dsr_msgs2/srv/aux_control/GetCurrentPosx.srv`

```srv
#____________________________________________________________________________________________
# get_current_posx()  
#____________________________________________________________________________________________
# This service returns the current task position.

int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
std_msgs/Float64MultiArray[] task_pos_info  # task pos = task_pos_info[0][0:5], solution sapce = task_pos_info[0][6]
bool        success
```

#### `aux_control/get_current_tool_flange_posx` (`dsr_msgs2/srv/GetCurrentToolFlangePosx`)

원문: `dsr_msgs2/srv/aux_control/GetCurrentToolFlangePosx.srv`

```srv
#____________________________________________________________________________________________
# get_current_tool_flange_posx()  
#____________________________________________________________________________________________
# This service returns the pose of the current tool flange based on the ref coordinate. In other words, it means the return to tcp=(0,0,0,0,0,0).

int8        ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6]  pos               # Pose of tool flange(posx) 
bool        success
```

#### `aux_control/get_current_velx` (`dsr_msgs2/srv/GetCurrentVelx`)

원문: `dsr_msgs2/srv/aux_control/GetCurrentVelx.srv`

```srv
#____________________________________________________________________________________________
# get_current_velx(ref)  
#____________________________________________________________________________________________
# This service returns the current tool velocity based on the ref coordinate.

int8       ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6] vel               # Tool velocity
bool       success
```

#### `aux_control/get_desired_posx` (`dsr_msgs2/srv/GetDesiredPosx`)

원문: `dsr_msgs2/srv/aux_control/GetDesiredPosx.srv`

```srv
#____________________________________________________________________________________________
# get_desired_posx(ref)  
#____________________________________________________________________________________________
# This service returns the target pose of the current tool. The pose is based on the ref coordinate.

int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
float64[6] pos               # task pos(posx)
bool       success
```

#### `aux_control/get_desired_velx` (`dsr_msgs2/srv/GetDesiredVelx`)

원문: `dsr_msgs2/srv/aux_control/GetDesiredVelx.srv`

```srv
#____________________________________________________________________________________________
# get_desired_velx(ref)  
#____________________________________________________________________________________________
# This service returns the target velocity of the current tool based on the ref coordinate. 
# It cannot be used in the movej, movejx, or movesj service.

int8       ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6] vel               # Tool velocity
bool       success
```

#### `aux_control/get_current_solution_space` (`dsr_msgs2/srv/GetCurrentSolutionSpace`)

원문: `dsr_msgs2/srv/aux_control/GetCurrentSolutionSpace.srv`

```srv
#____________________________________________________________________________________________
# get_current_solution_space
#____________________________________________________________________________________________
# This service returns the current solution space value.

---
int8        sol_space         # solution space : 0 ~ 7
bool        success
```

#### `aux_control/get_current_rotm` (`dsr_msgs2/srv/GetCurrentRotm`)

원문: `dsr_msgs2/srv/aux_control/GetCurrentRotm.srv`

```srv
#____________________________________________________________________________________________
# get_current_rotm(ref)  
#____________________________________________________________________________________________
# This service returns the direction and matrix of the current tool based on the ref coordinate.

int8        ref               # DR_BASE(0), DR_WORLD(2)
---
std_msgs/Float64MultiArray[] rot_matrix  # target[3][3] Rotation matrix
bool        success
```

#### `aux_control/get_joint_torque` (`dsr_msgs2/srv/GetJointTorque`)

원문: `dsr_msgs2/srv/aux_control/GetJointTorque.srv`

```srv
#____________________________________________________________________________________________
# get_joint_torque()
# returns the sensor torque value of the current joint.
#____________________________________________________________________________________________
# This service returns the sensor torque value of the current joint.

---
float64[6] jts         # value of JTS(Joint Torque Sensor) 
bool       success
```

#### `aux_control/get_external_torque` (`dsr_msgs2/srv/GetExternalTorque`)

원문: `dsr_msgs2/srv/aux_control/GetExternalTorque.srv`

```srv
#____________________________________________________________________________________________
# get_external_torque()
# returns the torque value generated by the external force on each current joint.
#____________________________________________________________________________________________
# This service returns the torque value generated by the external force on each current joint.

---
float64[6] ext_torque       #Torque value generated by an external force
bool       success
```

#### `aux_control/get_tool_force` (`dsr_msgs2/srv/GetToolForce`)

원문: `dsr_msgs2/srv/aux_control/GetToolForce.srv`

```srv
#____________________________________________________________________________________________
# get_tool_force(ref)
# returns the external force applied to the current tool
#____________________________________________________________________________________________
# This service returns the external force applied to the current tool based on the ref coordinate. 
# The force is based on the base coordinate while the moment is based on the tool coordinate.

int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
---
float64[6] tool_force        # External force applied to the tool
bool       success
```

#### `aux_control/get_solution_space` (`dsr_msgs2/srv/GetSolutionSpace`)

원문: `dsr_msgs2/srv/aux_control/GetSolutionSpace.srv`

```srv
#____________________________________________________________________________________________
# get_solution_space(pos)  
#____________________________________________________________________________________________
# This service obtains the solution space value.

float64[6] pos               # joint angle list [degree] 
---
int8       sol_space         # solution space : 0 ~ 7
bool       success
```

#### `aux_control/get_orientation_error` (`dsr_msgs2/srv/GetOrientationError`)

원문: `dsr_msgs2/srv/aux_control/GetOrientationError.srv`

```srv
#____________________________________________________________________________________________
# get_orientation_error  
#____________________________________________________________________________________________
# This service returns the orientation error value between the arbitrary poses xd and xc of the axis.

float64[6] xd                # task pos(posx)  
float64[6] xc                # task pos(posx)  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
---
float32    ori_error         # orientation error  
bool       success
```

#### `aux_control/get_robot_link_info` (`dsr_msgs2/srv/GetRobotLinkInfo`)

원문: `dsr_msgs2/srv/system/GetRobotLinkInfo.srv`

```srv
---
float32[6] d
float32[6] a
float32[6] alpha
float32[6] theta
float32[6] offset
float32 gradient
float32 rotation
bool success
```

#### `force/parallel_axis1` (`dsr_msgs2/srv/ParallelAxis1`)

원문: `dsr_msgs2/srv/force/ParallelAxis1.srv`

```srv
#____________________________________________________________________________________________
# parallel_axis(x1, x2, x3, axis, ref)  
#____________________________________________________________________________________________

float64[6] x1                # task pos(posx)  
float64[6] x2                # task pos(posx)  
float64[6] x3                # task pos(posx)
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
bool       success
```

#### `force/parallel_axis2` (`dsr_msgs2/srv/ParallelAxis2`)

원문: `dsr_msgs2/srv/force/ParallelAxis2.srv`

```srv
#____________________________________________________________________________________________
# parallel_axis(vect, axis, ref)  
#____________________________________________________________________________________________

float64[3] vect              # vector[3]  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
bool       success
```

#### `force/align_axis1` (`dsr_msgs2/srv/AlignAxis1`)

원문: `dsr_msgs2/srv/force/AlignAxis1.srv`

```srv
#____________________________________________________________________________________________
# align_axis(x1, x2, x3, pos, axis, ref)
#____________________________________________________________________________________________

float64[6] x1                # task pos(posx)  
float64[6] x2                # task pos(posx)  
float64[6] x3                # task pos(posx)
float64[3] source_vect       # source vector[3]  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
bool       success
```

#### `force/align_axis2` (`dsr_msgs2/srv/AlignAxis2`)

원문: `dsr_msgs2/srv/force/AlignAxis2.srv`

```srv
#____________________________________________________________________________________________
# align_axis(vect, pos, axis, ref)
#____________________________________________________________________________________________

float64[3] target_vect       # target vector[3]  
float64[3] source_vect       # source vector[3]  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
bool       success
```

#### `force/is_done_bolt_tightening` (`dsr_msgs2/srv/IsDoneBoltTightening`)

원문: `dsr_msgs2/srv/force/IsDoneBoltTightening.srv`

```srv
#____________________________________________________________________________________________
# is_done_bolt_tightening  
#____________________________________________________________________________________________

float64    m                 # Target torque  
float64    timeout           # Monitoring duration [sec]  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
---
bool       success
```

#### `force/release_compliance_ctrl` (`dsr_msgs2/srv/ReleaseComplianceCtrl`)

원문: `dsr_msgs2/srv/force/ReleaseComplianceCtrl.srv`

```srv
#____________________________________________________________________________________________
# release_compliance_ctrl  
#____________________________________________________________________________________________

---
bool       success
```

#### `force/task_compliance_ctrl` (`dsr_msgs2/srv/TaskComplianceCtrl`)

원문: `dsr_msgs2/srv/force/TaskComplianceCtrl.srv`

```srv
#____________________________________________________________________________________________
# task_compliance_ctrl  
#____________________________________________________________________________________________

float64[6] stx               # Three translational stiffnesses + Three rotational stiffnesses
                             # default  [3000, 3000, 3000, 200, 200, 200]
int8       ref               # the preset reference coordinate system.
float64    time              # Stiffness varying time [ 0 ~ 1.0 sec], Linear transition during the specified time 
---
bool       success
```

#### `force/set_stiffnessx` (`dsr_msgs2/srv/SetStiffnessx`)

원문: `dsr_msgs2/srv/force/SetStiffnessx.srv`

```srv
#____________________________________________________________________________________________
# set_stiffnessx  
#____________________________________________________________________________________________

float64[6] stx               # default[500, 500, 500, 100, 100, 100], Three translational stiffnesses + Three rotational stiffnesses
int8       ref               # the preset reference coordinate system.
float64    time              # Stiffness varying time(0 ~ 1.0) [sec], Linear transition during the specified time   
---
bool       success
```

#### `force/calc_coord` (`dsr_msgs2/srv/CalcCoord`)

원문: `dsr_msgs2/srv/force/CalcCoord.srv`

```srv
#____________________________________________________________________________________________
# calc_coord   
#____________________________________________________________________________________________
# This service is only available in M2.50 or later

int8       input_pos_cnt     # input_pos_cnt
float64[6] x1                # task pos(posx)  
float64[6] x2                # task pos(posx)  
float64[6] x3                # task pos(posx)
float64[6] x4                # task pos(posx)
int8       ref               # DR_BASE(0), DR_WORLD(2)
int8       mod               # input mode(only valid when the number of input poses is 2)
                             # 0: defining z-axis based on the current Tool-z direction
                             # 1: defining z-axis based on the z direction of x1 
---
float64[6] conv_posx         # task pos(posx) 
bool       success
```

#### `force/set_user_cart_coord1` (`dsr_msgs2/srv/SetUserCartCoord1`)

원문: `dsr_msgs2/srv/force/SetUserCartCoord1.srv`

```srv
#____________________________________________________________________________________________
# set_user_cart_coord(pos, ref)   
#____________________________________________________________________________________________

float64[6] pos                # task pos(posx)  
int8       ref                # DR_BASE(0), DR_WORLD(2)
                              # <ref is only available in M2.40 or later> 
---
int8    id                    # set user coord (101~120) or fail(-1)
bool        success
```

#### `force/set_user_cart_coord2` (`dsr_msgs2/srv/SetUserCartCoord2`)

원문: `dsr_msgs2/srv/force/SetUserCartCoord2.srv`

```srv
#____________________________________________________________________________________________
# set_user_cart_coord(x1, x2, x3, pos, ref)
#____________________________________________________________________________________________

float64[6] x1                 # task pos(posx)  
float64[6] x2                 # task pos(posx)  
float64[6] x3                 # task pos(posx)
float64[6] pos                # pos(posx)
int8       ref                # DR_BASE(0), DR_WORLD(2)
                              # <ref is only available in M2.40 or later> 
---
int8    id                    # set user coord (101~200) or fail(-1) 
bool        success
```

#### `force/set_user_cart_coord3` (`dsr_msgs2/srv/SetUserCartCoord3`)

원문: `dsr_msgs2/srv/force/SetUserCartCoord3.srv`

```srv
#____________________________________________________________________________________________
# set_user_cart_coord(u1, v1, pos, ref) 
#____________________________________________________________________________________________

float64[3] u1                # X-axis unit vector  
float64[3] v1                # Y-axis unit vector 
float64[6] pos               # task pos(posx) 
int8       ref               # DR_BASE(0), DR_WORLD(2)
                             # <ref is only available in M2.40 or later> 
---
int8    id                   # set user coord (101~120) or fail(-1) 
bool        success
```

#### `force/overwrite_user_cart_coord` (`dsr_msgs2/srv/OverwriteUserCartCoord`)

원문: `dsr_msgs2/srv/force/OverwriteUserCartCoord.srv`

```srv
#____________________________________________________________________________________________
# overwrite_user_cart_coord   
#____________________________________________________________________________________________
# This service is only available in M2.50 or later

int8       id                # ID of user coord 
float64[6] pos               # task pos(posx)  
int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2)
---
int8       id                # Successful coordinate setting, Set user coordinate ID (101 - 200)
                             # (-1) Failed coordinate setting
bool       success
```

#### `force/get_user_cart_coord` (`dsr_msgs2/srv/GetUserCartCoord`)

원문: `dsr_msgs2/srv/force/GetUserCartCoord.srv`

```srv
#____________________________________________________________________________________________
# posx, ref = get_user_cart_coord(id)   
#____________________________________________________________________________________________
# This service is only available in M2.50 or later

int8       id                # ID of user coord 
---
float64[6] conv_posx         # task pos(posx)  
int8       ref               # Reference coordinate of the coordinate to get
bool       success
```

#### `force/set_desired_force` (`dsr_msgs2/srv/SetDesiredForce`)

원문: `dsr_msgs2/srv/force/SetDesiredForce.srv`

```srv
#____________________________________________________________________________________________
# set_desired_force  
#____________________________________________________________________________________________

float64[6] fd                # Three translational target forces + Three rotational target moments
int8[6]    dir               # Force control in the corresponding direction if 1, Compliance control in the corresponding direction if 0
int8       ref               # Reference coordinate of the coordinate to get
float64    time # 0          # Transition time of target force to take effect (0 ~ 1.0 sec)
int8       mod               # DR_FC_MOD_ABS(0): force control with absolute value, 
                             # DR_FC_MOD_REL(1): force control with relative value to initial state (the instance when this function is called) 
---
bool       success
```

#### `force/release_force` (`dsr_msgs2/srv/ReleaseForce`)

원문: `dsr_msgs2/srv/force/ReleaseForce.srv`

```srv
#____________________________________________________________________________________________
# release_force  
#____________________________________________________________________________________________

float64    time # 0          # Time needed to reduce the force (0 ~ 1.0) 
---
bool       success
```

#### `force/check_position_condition` (`dsr_msgs2/srv/CheckPositionCondition`)

원문: `dsr_msgs2/srv/force/CheckPositionCondition.srv`

```srv
#____________________________________________________________________________________________
# check_position_condition  
#____________________________________________________________________________________________

int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
float64    min               # min    
float64    max               # max  
int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user_coordinate(101~200)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0         # DR_MV_MOD_ABS(0), DR_MV_MOD_REL(1) 
float64[6] pos               # task pos(posx)  
---
bool success                 # True or False
```

#### `force/check_force_condition` (`dsr_msgs2/srv/CheckForceCondition`)

원문: `dsr_msgs2/srv/force/CheckForceCondition.srv`

```srv
#____________________________________________________________________________________________
# check_force_condition 
#This service checks the status of the given force. It disregards the force direction and only compares the sizes. 
#This condition can be repeated with the while or if statement. Measuring the force, axis is based on the ref coordinate and measuring the moment,
#axis is based on the tool coordinate.
#____________________________________________________________________________________________

int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2), DR_AXIS_A(10), DR_AXIS_B(11), DR_AXIS_C(12) 
float64    min               # min >=0.0   
float64    max               # max >=0.0 
int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200)
                             # <DR_WORLD is only available in M2.40 or later> 
---
bool       success                 # True or False
```

#### `force/check_orientation_condition1` (`dsr_msgs2/srv/CheckOrientationCondition1`)

원문: `dsr_msgs2/srv/force/CheckOrientationCondition1.srv`

```srv
#____________________________________________________________________________________________
# check_orientation_condition(axis, min, max, ref, mod)  
#____________________________________________________________________________________________

int8       axis              # DR_AXIS_A(10), DR_AXIS_B(11), DR_AXIS_C(12) 
float64[6] min               # task pos(posx)  
float64[6] max               # task pos(posx)  
int8       ref  #= 0         # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user_coordinate(101~200)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0         # DR_MV_MOD_ABS(0)
---
bool success                 # True or False
```

#### `force/check_orientation_condition2` (`dsr_msgs2/srv/CheckOrientationCondition2`)

원문: `dsr_msgs2/srv/force/CheckOrientationCondition2.srv`

```srv
#____________________________________________________________________________________________
# check_orientation_condition(axis, min, max, ref, mod, pos)  
#____________________________________________________________________________________________

int8       axis              # DR_AXIS_A(10), DR_AXIS_B(11), DR_AXIS_C(12) 
float64    min               # minimum value  
float64    max               # maximum value  
int8       ref  #= 0         # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user_coordinate(101~200)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 1         # DR_MV_MOD_REL(1)
float64[6] pos               # task pos(pos)  
---
bool success                 # True or False
```

#### `force/coord_transform` (`dsr_msgs2/srv/CoordTransform`)

원문: `dsr_msgs2/srv/force/CoordTransform.srv`

```srv
#____________________________________________________________________________________________
# coord_transform   
#____________________________________________________________________________________________

float64[6] pos_in            # task pos(posx)  
int8       ref_in            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
int8       ref_out           # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200) 
                             # <ref is only available in M2.40 or later> 
---
float64[6] conv_posx         # task pos(posx)
bool       success
```

#### `force/get_workpiece_weight` (`dsr_msgs2/srv/GetWorkpieceWeight`)

원문: `dsr_msgs2/srv/force/GetWorkpieceWeight.srv`

```srv
#____________________________________________________________________________________________
# get_workpiece_weight
#____________________________________________________________________________________________

---
float32       weight               # Measured weight, Negative value if error
bool          success
```

#### `force/reset_workpiece_weight` (`dsr_msgs2/srv/ResetWorkpieceWeight`)

원문: `dsr_msgs2/srv/force/ResetWorkpieceWeight.srv`

```srv
#____________________________________________________________________________________________
# reset_workpiece_weight
# Initializes the weight data of the material to initialize the algorithm before measuring the weight of the material.
# ____________________________________________________________________________________________

---
bool       success
```

#### `io/set_ctrl_box_digital_output` (`dsr_msgs2/srv/SetCtrlBoxDigitalOutput`)

원문: `dsr_msgs2/srv/io/SetCtrlBoxDigitalOutput.srv`

```srv
#____________________________________________________________________________________________
# set_digital_output  
#____________________________________________________________________________________________

int8       index    # ctrlbox digital output port(1 ~ 16)
int8       value    # 0 : ON, 1 : OFF
---
bool        success
```

#### `io/get_ctrl_box_digital_input` (`dsr_msgs2/srv/GetCtrlBoxDigitalInput`)

원문: `dsr_msgs2/srv/io/GetCtrlBoxDigitalInput.srv`

```srv
#____________________________________________________________________________________________
# get_digital_input
# This service reads the I/O signals from digital contact points of the controller and reads the digital input contact value.
#____________________________________________________________________________________________

int8        index    # Digital Input in Control Box(1 ~ 16) 
                     # <GPIO_CTRLBOX_DIGITAL_INDEX>
---
int8        value    # 0=OFF, 1=ON
bool        success
```

#### `io/set_tool_digital_output` (`dsr_msgs2/srv/SetToolDigitalOutput`)

원문: `dsr_msgs2/srv/io/SetToolDigitalOutput.srv`

```srv
#____________________________________________________________________________________________
# set_tool_digital_output  
# This service sends ouptput to tool io.
#____________________________________________________________________________________________

int8       index   # flange digital output port(1 ~ 6)
int8       value   # 0 : ON, 1 : OFF
---
bool       success
```

#### `io/get_tool_digital_input` (`dsr_msgs2/srv/GetToolDigitalInput`)

원문: `dsr_msgs2/srv/io/GetToolDigitalInput.srv`

```srv
#____________________________________________________________________________________________
# get_tool_digital_input  
# This service gets the current controlbox io output status.
#____________________________________________________________________________________________

int8        index    # Digital Input in Flange(1 ~ 6) 
                     # <GPIO_TOOL_DIGITAL_INDEX> 
---
int8        value    # 0=OFF, 1=ON
bool        success
```

#### `io/set_ctrl_box_analog_output` (`dsr_msgs2/srv/SetCtrlBoxAnalogOutput`)

원문: `dsr_msgs2/srv/io/SetCtrlBoxAnalogOutput.srv`

```srv
#____________________________________________________________________________________________
# set_ctrl_box_analog_output  
#____________________________________________________________________________________________

int8        channel  # 1 = ch1, 2= ch2 
float64     value   #
---
bool        success
```

#### `io/get_ctrl_box_analog_input` (`dsr_msgs2/srv/GetCtrlBoxAnalogInput`)

원문: `dsr_msgs2/srv/io/GetCtrlBoxAnalogInput.srv`

```srv
#____________________________________________________________________________________________
# get_analog_input 
#____________________________________________________________________________________________

int8        channel    # 1 = ch1, 2= ch2
---
float64     value
bool        success
```

#### `io/set_ctrl_box_analog_output_type` (`dsr_msgs2/srv/SetCtrlBoxAnalogOutputType`)

원문: `dsr_msgs2/srv/io/SetCtrlBoxAnalogOutputType.srv`

```srv
#____________________________________________________________________________________________
# set_ctrl_box_analog_output_type  
#____________________________________________________________________________________________

int8        channel  # 1 = ch1, 2= ch2 
int8        mode     # 0 = current, 1 = voltage
---
bool        success
```

#### `io/set_ctrl_box_analog_input_type` (`dsr_msgs2/srv/SetCtrlBoxAnalogInputType`)

원문: `dsr_msgs2/srv/io/SetCtrlBoxAnalogInputType.srv`

```srv
#____________________________________________________________________________________________
# set_ctrl_box_analog_input_type  
#____________________________________________________________________________________________

int8        channel  # 1 = ch1, 2= ch2 
int8        mode     # 0 = current, 1 = voltage
---
bool        success
```

#### `io/get_ctrl_box_digital_output` (`dsr_msgs2/srv/GetCtrlBoxDigitalOutput`)

원문: `dsr_msgs2/srv/io/GetCtrlBoxDigitalOutput.srv`

```srv
#____________________________________________________________________________________________
# get_digital_output  
#____________________________________________________________________________________________

int8       index    # ctrlbox digital output port(1 ~ 16)
---
int8       value    # Current output status (0 : ON, 1 : OFF)
bool       success
```

#### `io/get_tool_digital_output` (`dsr_msgs2/srv/GetToolDigitalOutput`)

원문: `dsr_msgs2/srv/io/GetToolDigitalOutput.srv`

```srv
#____________________________________________________________________________________________
# get_tool_digital_output  
# This service gets the current tool io output status.
#____________________________________________________________________________________________

int8       index   # flange digital output port(1 ~ 6)
---
int8       value   # Current output status (0 : ON, 1 : OFF)
bool       success
```

#### `modbus/set_modbus_output` (`dsr_msgs2/srv/SetModbusOutput`)

원문: `dsr_msgs2/srv/modbus/SetModbusOutput.srv`

```srv
#____________________________________________________________________________________________
# set_modbus_output  
# This service sends the signal to an external Modbus system. 
#____________________________________________________________________________________________

string      name     # modbus signal symbol
int32       value    # modbus register value
---
bool        success
```

#### `modbus/get_modbus_input` (`dsr_msgs2/srv/GetModbusInput`)

원문: `dsr_msgs2/srv/modbus/GetModbusInput.srv`

```srv
#____________________________________________________________________________________________
# get_modbus_input  
# This service reads the signal from the Modbus system.
#____________________________________________________________________________________________

string      name    # modbus signal symbol
---
int32       value    # modbus signal value
bool        success
```

#### `modbus/config_create_modbus` (`dsr_msgs2/srv/ConfigCreateModbus`)

원문: `dsr_msgs2/srv/modbus/ConfigCreateModbus.srv`

```srv
#____________________________________________________________________________________________
# config_create_modbus  
# This service registers the Modbus signal. 
#____________________________________________________________________________________________

string      name       # modbus signal symbol 
string      ip         # external device ip
int32       port       # external device port     
int8        reg_type   # <MODBUS_REGISTER_TYPE>(0: discrete input, 1: coil, 2: input register, 3: holding register)
int8        index      # modbus signal index(0 ~ 9999)
int8        value      # modbus singla value(unsigned value ; 0 ~ 65535)
int32       slave_id   # Slave ID of the ModbusTCP(0: Broadcase address or 1-247 or 255: Default value for ModbusTCP) 
                       # <slave_id is only available in M2.40 or later versions>  
---
bool success
```

#### `modbus/config_delete_modbus` (`dsr_msgs2/srv/ConfigDeleteModbus`)

원문: `dsr_msgs2/srv/modbus/ConfigDeleteModbus.srv`

```srv
#____________________________________________________________________________________________
# config_delete_modbus 
# It is a service to delete the Modbus I / O signal information registered 
# in advance in the robot controller 
#____________________________________________________________________________________________

string      name       # modbus signal symbol 
---
bool success
```

#### `tcp/config_create_tcp` (`dsr_msgs2/srv/ConfigCreateTcp`)

원문: `dsr_msgs2/srv/tcp/ConfigCreateTcp.srv`

```srv
#____________________________________________________________________________________________
# config_create_tcp  
# It is a service for registering and using robot TCP information in advance for safety
#____________________________________________________________________________________________

string          name         # tcp name 
float64[6]      pos          # coordinates of the TCP 
---
bool success
```

#### `tcp/config_delete_tcp` (`dsr_msgs2/srv/ConfigDeleteTcp`)

원문: `dsr_msgs2/srv/tcp/ConfigDeleteTcp.srv`

```srv
#____________________________________________________________________________________________
# config_delete_tcp  
# It is a service for deleting the TCP information registered in advance in the robot controller
#____________________________________________________________________________________________

string          name             # tcp name 
---
bool success
```

#### `tcp/get_current_tcp` (`dsr_msgs2/srv/GetCurrentTcp`)

원문: `dsr_msgs2/srv/tcp/GetCurrentTcp.srv`

```srv
#____________________________________________________________________________________________
# get_current_tcp  
# It is the service to get the currently set TCP information from the robot controller
#____________________________________________________________________________________________

---
string         info # tcp name
bool        success
```

#### `tcp/set_current_tcp` (`dsr_msgs2/srv/SetCurrentTcp`)

원문: `dsr_msgs2/srv/tcp/SetCurrentTcp.srv`

```srv
#____________________________________________________________________________________________
# set_current_tcp  
# It is a service that sets the information about the currently installed TCP
#____________________________________________________________________________________________

string         name # tcp name
---
bool           success
```

#### `tool/config_create_tool` (`dsr_msgs2/srv/ConfigCreateTool`)

원문: `dsr_msgs2/srv/tool/ConfigCreateTool.srv`

```srv
#____________________________________________________________________________________________
# config_create_tool 
# It is a service for registering and using robot Tool information in advance for safety 
#____________________________________________________________________________________________

string          name        # tool name 
float64         weight      # tool weight 
float64[3]      cog         # Center of gravity
float64[6]      inertia     # tool inertia 
---
bool success
```

#### `tool/config_delete_tool` (`dsr_msgs2/srv/ConfigDeleteTool`)

원문: `dsr_msgs2/srv/tool/ConfigDeleteTool.srv`

```srv
#____________________________________________________________________________________________
# config_delete_tool  
# It is a service to delete tool information registered in advance in the robot controller
#____________________________________________________________________________________________

string          name        # tool name 
---
bool success
```

#### `tool/get_current_tool` (`dsr_msgs2/srv/GetCurrentTool`)

원문: `dsr_msgs2/srv/tool/GetCurrentTool.srv`

```srv
#____________________________________________________________________________________________
# get_current_tool  
# It is a service to fetch the currently set tool information from the robot controller
#____________________________________________________________________________________________

---
string         info # tool name
bool        success
```

#### `tool/set_current_tool` (`dsr_msgs2/srv/SetCurrentTool`)

원문: `dsr_msgs2/srv/tool/SetCurrentTool.srv`

```srv
#____________________________________________________________________________________________
# set_current_tool
# It is a service to set information about currently installed tool.  
#____________________________________________________________________________________________

string          name        # tool name
---
bool            success
```

#### `tool/set_tool_shape` (`dsr_msgs2/srv/SetToolShape`)

원문: `dsr_msgs2/srv/tool/SetToolShape.srv`

```srv
#____________________________________________________________________________________________
# set_tool_shape
# It is a service to set information about currently installed tool.  
# Activates the tool shape information of the entered name among the tool shape information registered in the Teach Pendant
#____________________________________________________________________________________________

string          name        # Tool name registered in the Teach Pendant
---
bool            success
```

#### `drl/drl_pause` (`dsr_msgs2/srv/DrlPause`)

원문: `dsr_msgs2/srv/drl/DrlPause.srv`

```srv
#____________________________________________________________________________________________
# drl_script_pause  
# This service is used to stop the currently executing DRL program from the robot controller.
#____________________________________________________________________________________________

---
bool success
```

#### `drl/drl_start` (`dsr_msgs2/srv/DrlStart`)

원문: `dsr_msgs2/srv/drl/DrlStart.srv`

```srv
#____________________________________________________________________________________________
# drl_script_run  
# This is a service to execute a program configured in the DRL language in the robot controller.
#____________________________________________________________________________________________

int8 robot_system    # Robot System Mode 0 : Real, 1 : virtual
string  code        # drl code       
---
bool success
```

#### `drl/drl_stop` (`dsr_msgs2/srv/DrlStop`)

원문: `dsr_msgs2/srv/drl/DrlStop.srv`

```srv
#____________________________________________________________________________________________
# drl_script_stop  
# STOP_TYPE_QUICK_STO = 0
# STOP_TYPE_QUICK     = 1
# STOP_TYPE_SLOW      = 2
# STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY = 3  
#____________________________________________________________________________________________

int8    stop_mode       # <STOP_TYPE> stop_mode       
---
bool    success
```

#### `drl/drl_resume` (`dsr_msgs2/srv/DrlResume`)

원문: `dsr_msgs2/srv/drl/DrlResume.srv`

```srv
#____________________________________________________________________________________________
# drl_script_resume  
# It is a service to resume the currently paused DRL program in the robot controller.    
#____________________________________________________________________________________________

---
bool success
```

#### `drl/get_drl_state` (`dsr_msgs2/srv/GetDrlState`)

원문: `dsr_msgs2/srv/drl/GetDrlState.srv`

```srv
#____________________________________________________________________________________________
# get_drl_state
# Get DRL Program State
# 0 : DRL_PROGRAM_STATE_PLAY
# 1 : DRL_PROGRAM_STATE_STOP
# 2 : DRL_PROGRAM_STATE_HOLD
# 3 : DRL_PROGRAM_STATE_LAST
# drfl.GetProgramState()
#____________________________________________________________________________________________

---
int8        drl_state # <DRL_PROGRAM_STATE>
bool        success
```

#### `realtime/connect_rt_control` (`dsr_msgs2/srv/ConnectRtControl`)

원문: `dsr_msgs2/srv/realtime/ConnectRtControl.srv`

```srv
#____________________________________________________________________________________________
# connect_rt_control
#____________________________________________________________________________________________

string     ip_address
uint32     port
---
bool       success
```

#### `realtime/disconnect_rt_control` (`dsr_msgs2/srv/DisconnectRtControl`)

원문: `dsr_msgs2/srv/realtime/DisconnectRtControl.srv`

```srv
#____________________________________________________________________________________________
# disconnect_rt_control
#____________________________________________________________________________________________
---
bool       success
```

#### `realtime/get_rt_control_output_version_list` (`dsr_msgs2/srv/GetRtControlOutputVersionList`)

원문: `dsr_msgs2/srv/realtime/GetRtControlOutputVersionList.srv`

```srv
#____________________________________________________________________________________________
# get_rt_control_output_version_list
#____________________________________________________________________________________________

---
bool       success
string     version
```

#### `realtime/get_rt_control_input_version_list` (`dsr_msgs2/srv/GetRtControlInputVersionList`)

원문: `dsr_msgs2/srv/realtime/GetRtControlInputVersionList.srv`

```srv
#____________________________________________________________________________________________
# get_rt_control_input_version_list
#____________________________________________________________________________________________

---
bool       success
string     version
```

#### `realtime/get_rt_control_input_data_list` (`dsr_msgs2/srv/GetRtControlInputDataList`)

원문: `dsr_msgs2/srv/realtime/GetRtControlInputDataList.srv`

```srv
#____________________________________________________________________________________________
# get_rt_control_input_data_list
#____________________________________________________________________________________________
string     version
---
bool       success
string     data
```

#### `realtime/get_rt_control_output_data_list` (`dsr_msgs2/srv/GetRtControlOutputDataList`)

원문: `dsr_msgs2/srv/realtime/GetRtControlOutputDataList.srv`

```srv
#____________________________________________________________________________________________
# get_rt_control_output_data_list
#____________________________________________________________________________________________

string     version
---
bool       success
string     data
```

#### `realtime/set_rt_control_input` (`dsr_msgs2/srv/SetRtControlInput`)

원문: `dsr_msgs2/srv/realtime/SetRtControlInput.srv`

```srv
#____________________________________________________________________________________________
# set_rt_control_input
#____________________________________________________________________________________________
string     version
float64    period
int32      loss
---
bool       success
```

#### `realtime/set_rt_control_output` (`dsr_msgs2/srv/SetRtControlOutput`)

원문: `dsr_msgs2/srv/realtime/SetRtControlOutput.srv`

```srv
#____________________________________________________________________________________________
# set_rt_control_output
#____________________________________________________________________________________________
string     version
float64    period
int32      loss
---
bool       success
```

#### `realtime/start_rt_control` (`dsr_msgs2/srv/StartRtControl`)

원문: `dsr_msgs2/srv/realtime/StartRtControl.srv`

```srv
#____________________________________________________________________________________________
# start_rt_control
#____________________________________________________________________________________________

---
bool       success
```

#### `realtime/stop_rt_control` (`dsr_msgs2/srv/StopRtControl`)

원문: `dsr_msgs2/srv/realtime/StopRtControl.srv`

```srv
#____________________________________________________________________________________________
# stop_rt_control
#____________________________________________________________________________________________
---
bool       success
```

#### `realtime/set_velj_rt` (`dsr_msgs2/srv/SetVeljRt`)

원문: `dsr_msgs2/srv/realtime/SetVeljRt.srv`

```srv
#____________________________________________________________________________________________
# set_velj_rt
#____________________________________________________________________________________________
float64[6] vel
---
bool       success
```

#### `realtime/set_accj_rt` (`dsr_msgs2/srv/SetAccjRt`)

원문: `dsr_msgs2/srv/realtime/SetAccjRt.srv`

```srv
#____________________________________________________________________________________________
# set_accj_rt
#____________________________________________________________________________________________
float64[6] acc
---
bool       success
```

#### `realtime/set_velx_rt` (`dsr_msgs2/srv/SetVelxRt`)

원문: `dsr_msgs2/srv/realtime/SetVelxRt.srv`

```srv
#____________________________________________________________________________________________
# set_velx_rt
#____________________________________________________________________________________________
float64    trans
float64    rotation
---
bool       success
```

#### `realtime/set_accx_rt` (`dsr_msgs2/srv/SetAccxRt`)

원문: `dsr_msgs2/srv/realtime/SetAccxRt.srv`

```srv
#____________________________________________________________________________________________
# set_accx_rt
#____________________________________________________________________________________________
float64    trans
float64    rotation
---
bool       success
```

#### `realtime/read_data_rt` (`dsr_msgs2/srv/ReadDataRt`)

원문: `dsr_msgs2/srv/realtime/ReadDataRt.srv`

```srv
#____________________________________________________________________________________________
# read_data_rt
#____________________________________________________________________________________________

---
RobotStateRt       data
```

#### `realtime/write_data_rt` (`dsr_msgs2/srv/WriteDataRt`)

원문: `dsr_msgs2/srv/realtime/WriteDataRt.srv`

```srv
#____________________________________________________________________________________________
# write_data_rt
#____________________________________________________________________________________________
float64[6] external_force_torque
int32      external_digital_input
int32      external_digital_output
float64[6] external_analog_input
float64[6] external_analog_output
---
bool       success
```

#### `plc/get_input_register_int` (`dsr_msgs2/srv/GetInputRegisterInt`)

원문: `dsr_msgs2/srv/plc/GetInputRegisterInt.srv`

```srv
uint16 address
uint32 timeout_ms
---
bool success
int32 value
```

#### `plc/get_input_register_bit` (`dsr_msgs2/srv/GetInputRegisterBit`)

원문: `dsr_msgs2/srv/plc/GetInputRegisterBit.srv`

```srv
uint16 address
uint32 timeout_ms
---
bool success
int32 value
```

#### `plc/get_input_register_float` (`dsr_msgs2/srv/GetInputRegisterFloat`)

원문: `dsr_msgs2/srv/plc/GetInputRegisterFloat.srv`

```srv
uint16 address
uint32 timeout_ms
---
bool success
float64 value
```

#### `plc/set_output_register_int` (`dsr_msgs2/srv/SetOutputRegisterInt`)

원문: `dsr_msgs2/srv/plc/SetOutputRegisterInt.srv`

```srv
uint16 address
int32 value
---
bool success
```

#### `plc/set_output_register_bit` (`dsr_msgs2/srv/SetOutputRegisterBit`)

원문: `dsr_msgs2/srv/plc/SetOutputRegisterBit.srv`

```srv
uint16 address
int32 value
---
bool success
```

#### `plc/set_output_register_float` (`dsr_msgs2/srv/SetOutputRegisterFloat`)

원문: `dsr_msgs2/srv/plc/SetOutputRegisterFloat.srv`

```srv
uint16 address
float64 value
---
bool success
```

#### `plc/get_output_register_int` (`dsr_msgs2/srv/GetOutputRegisterInt`)

원문: `dsr_msgs2/srv/plc/GetOutputRegisterInt.srv`

```srv
uint16 address
uint32 timeout_ms
---
bool success
int32 value
```

#### `plc/get_output_register_bit` (`dsr_msgs2/srv/GetOutputRegisterBit`)

원문: `dsr_msgs2/srv/plc/GetOutputRegisterBit.srv`

```srv
uint16 address
uint32 timeout_ms
---
bool success
int32 value
```

#### `plc/get_output_register_float` (`dsr_msgs2/srv/GetOutputRegisterFloat`)

원문: `dsr_msgs2/srv/plc/GetOutputRegisterFloat.srv`

```srv
uint16 address
uint32 timeout_ms
---
bool success
float64 value
```

### 11.3 Topic message 원문 필드

#### `alter_motion_stream` (`dsr_msgs2/msg/AlterMotionStream`, sub depth 20)

원문: `dsr_msgs2/msg/AlterMotionStream.msg`

```msg
#____________________________________________________________________________________________
# alter_motion  
# 
#____________________________________________________________________________________________

float64[6] pos               # position
```

#### `servoj_stream` (`dsr_msgs2/msg/ServojStream`, sub depth 20)

원문: `dsr_msgs2/msg/ServojStream.msg`

```msg
#____________________________________________________________________________________________
# servoj
# 
#____________________________________________________________________________________________

float64[6] pos               # position  
float64[6] vel               # velocity
float64[6] acc               # acceleration
float64    time              # time
int8       mode              # servoj mode; 0:DR_SERVO_OVERRIDE, 1:DR_SERVO_QUEUE
```

#### `servol_stream` (`dsr_msgs2/msg/ServolStream`, sub depth 20)

원문: `dsr_msgs2/msg/ServolStream.msg`

```msg
#____________________________________________________________________________________________
# servol
# 
#____________________________________________________________________________________________

float64[6] pos               # position  
float64[2] vel               # velocity
float64[2] acc               # acceleration
float64    time              # time
```

#### `speedj_stream` (`dsr_msgs2/msg/SpeedjStream`, sub depth 20)

원문: `dsr_msgs2/msg/SpeedjStream.msg`

```msg
#____________________________________________________________________________________________
# speedj
# 
#____________________________________________________________________________________________

float64[6] vel               # velocity
float64[6] acc               # acceleration
float64    time              # time
```

#### `speedl_stream` (`dsr_msgs2/msg/SpeedlStream`, sub depth 10)

원문: `dsr_msgs2/msg/SpeedlStream.msg`

```msg
#____________________________________________________________________________________________
# speedl
# 
#____________________________________________________________________________________________

float64[6] vel               # velocity
float64[2] acc               # acceleration
float64    time              # time
```

#### `servoj_rt_stream` (`dsr_msgs2/msg/ServojRtStream`, sub depth 20)

원문: `dsr_msgs2/msg/ServojRtStream.msg`

```msg
#____________________________________________________________________________________________
# servoj_rt
# 
#____________________________________________________________________________________________

float64[6] pos               # position  
float64[6] vel               # velocity
float64[6] acc               # acceleration
float64    time              # time
```

#### `servol_rt_stream` (`dsr_msgs2/msg/ServolRtStream`, sub depth 20)

원문: `dsr_msgs2/msg/ServolRtStream.msg`

```msg
#____________________________________________________________________________________________
# servol_rt
# 
#____________________________________________________________________________________________

float64[6] pos               # position  
float64[6] vel               # velocity
float64[6] acc               # acceleration
float64    time              # time
```

#### `speedj_rt_stream` (`dsr_msgs2/msg/SpeedjRtStream`, sub depth 20)

원문: `dsr_msgs2/msg/SpeedjRtStream.msg`

```msg
#____________________________________________________________________________________________
# speedj_rt
# 
#____________________________________________________________________________________________

float64[6] vel               # velocity
float64[6] acc               # acceleration
float64    time              # time
```

#### `speedl_rt_stream` (`dsr_msgs2/msg/SpeedlRtStream`, sub depth 20)

원문: `dsr_msgs2/msg/SpeedlRtStream.msg`

```msg
#____________________________________________________________________________________________
# speedl_rt
# 
#____________________________________________________________________________________________

float64[6] vel               # velocity
float64[6] acc               # acceleration
float64    time              # time
```

#### `torque_rt_stream` (`dsr_msgs2/msg/TorqueRtStream`, sub depth 20)

원문: `dsr_msgs2/msg/TorqueRtStream.msg`

```msg
#____________________________________________________________________________________________
# torque_rt
# 
#____________________________________________________________________________________________

float64[6] tor               # motor torque
float64    time              # time
```

#### `error` (`dsr_msgs2/msg/RobotError`, pub depth 100)

원문: `dsr_msgs2/msg/RobotError.msg`

```msg
#____________________________________________________________________________________________
# [ robot error msg ] 
#____________________________________________________________________________________________

int32    level   # INFO =1, WARN =2, ERROR =3 
int32    group   # SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   
int32    code    # error code 
string    msg1    # error msg 1
string    msg2    # error msg 2
string    msg3    # error msg 3
```

#### `robot_disconnection` (`dsr_msgs2/msg/RobotDisconnection`, pub depth 100)

원문: `dsr_msgs2/msg/RobotDisconnection.msg`

```msg
### Event driven when the robot connection losts.
```

### 11.4 Action 원문 필드

#### `motion/movej_h2r` (`dsr_msgs2/action/MovejH2r`)

원문: `dsr_msgs2/action/MovejH2r.action`

```action
# Goal
float64[6] target_pos
float64[6] target_vel
float64[6] target_acc
---
# Result
bool success
---
# Feedback
float64[6] pos
```

#### `motion/movel_h2r` (`dsr_msgs2/action/MovelH2r`)

원문: `dsr_msgs2/action/MovelH2r.action`

```action
# Goal
float64[6] target_pos
float64[2] target_vel
float64[2] target_acc
---
# Result
bool success
---
# Feedback
float64[6] pos
```
