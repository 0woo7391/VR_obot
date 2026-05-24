# RH-P12-RN ROS 2 AI 제어 명세

이 문서는 `RH-P12-RN` 원본 구성과 `RH-P12-RN-A` ROS 2 패키지의 제어 endpoint, 단위, QoS, 제약 사항만 정리한 AI 입력용 명세다.

배포 대상에는 ROS 2 구현 패키지 `RH-P12-RN-A`도 포함된다. 따라서 이 문서는 두 기준을 함께 다룬다.

- 원본 `RH-P12-RN`: ROS 1 기능 계약을 ROS 2로 포팅할 때의 기준
- `RH-P12-RN-A`: ROS 2 description/bringup/MoveIt/ros2_control 구성 기준

패키지별 별도 `_manual.md`를 만들지 않고 이 파일 하나에 통합한다.

## 절대 규칙

- 원본 `RH-P12-RN`은 ROS 1 패키지이므로 ROS 2에서 그대로 실행되는 구성이 아니다.
- 원본 저장소 내부에는 액션 정의, 액션 서버, 액션 클라이언트가 없다. 원본 포팅용 driver를 새로 만들 때는 특별한 요구가 없으면 action을 만들지 않는다.
- 단, `RH-P12-RN-A` ROS 2 구성은 `position_controllers/GripperActionController`를 사용하므로 `/gripper_controller/gripper_cmd` action이 표준 제어 입구다.
- 원본의 실제 제어 핵심은 단일 Dynamixel 장치 이름 `gripper`를 대상으로 하는 control table write/read다.
- 원본에서 실제 위치 명령은 `/robotis/direct/sync_write_item`로 들어온 `goal_position`만 `BaseModule`이 직접 처리한다.
- 원본에서 `/robotis/sync_write_item` subscriber는 이 저장소 안에 없다. 이것은 외부 ROBOTIS framework가 처리하는 인터페이스로 취급해야 한다.
- 원본의 `WriteCtrlItem.msg`는 정의만 있고 사용처가 없다. ROS 2에서 반드시 살릴 필요는 없다.
- 아래에 명시되지 않은 topic, service, action은 기본 인터페이스로 간주하지 않는다.

## 원본 포팅 시 핵심 기능

| 기능 | 원본 구현 | ROS 2 구현 기준 |
| --- | --- | --- |
| 그리퍼 위치 명령 | `/robotis/direct/sync_write_item` 구독 | `gripper`의 `goal_position` 명령을 받아 실제/시뮬레이션 backend에 전달 |
| 제어 테이블 쓰기 | GUI가 `/robotis/sync_write_item` 발행 | torque, mode, current, velocity, acceleration 쓰기 API 제공 |
| 상태 읽기 | `/robotis/rh_p12_rn_base/get_item_value` 서비스 | `gripper`의 상태/control table 값을 요청-응답으로 제공 |
| 현재 상태 발행 | present position/current topic | 현재 position/current를 ROS 2 topic으로 publish |
| RViz joint 변환 | `rviz_rh_pub` | 단일 구동 joint 값에서 보조 joint 값을 생성 |
| Gazebo joint 변환 | `gazebo_rh_pub` | 단일 command에서 보조 joint command 3개 생성 |

## 현재 ROS 2 패키지 구조: `RH-P12-RN-A`

| 패키지 | 목적 | 핵심 파일 |
| --- | --- | --- |
| `rh_p12_rn_a` | 메타/설치 패키지. 이 패키지 자체에는 실행 노드가 없다. | `package.xml`, `CMakeLists.txt` |
| `rh_p12_rn_a_description` | URDF/Xacro, Gazebo Xacro, ros2_control Xacro, robot_state_publisher launch | `urdf/rh_p12_rn_a.xacro`, `urdf/rh_p12_rn_a.urdf.xacro`, `ros2_control/rh_p12_rn_a.ros2_control.xacro`, `launch/rh_p12_rn_a.launch.py` |
| `rh_p12_rn_a_bringup` | 실제/가짜 hardware 및 Gazebo bringup, controller_manager 실행 | `launch/rh_p12_rn_a.launch.py`, `launch/rh_p12_rn_a_gazebo.launch.py`, `config/hardware_controller_manager.yaml` |
| `rh_p12_rn_a_moveit_config` | MoveIt 2 gripper controller 매핑, kinematics/planning/limit 설정 | `launch/rh_p12_rn_a_moveit.launch.py`, `config/moveit_controllers.yaml`, `config/joint_limits.yaml` |

현재 ROS 2 패키지의 기본 제어 방식은 `position_controllers/GripperActionController`이다. 직접 만든 custom service가 아니라 `/gripper_controller/gripper_cmd` action을 우선 사용한다.

### 현재 ROS 2 핵심 endpoint

| 분류 | 이름 | 타입 | 방향 | QoS / 단위 / 주의사항 |
| --- | --- | --- | --- | --- |
| Act | `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | controller server | action 기본 QoS; `command.position`은 `rh_r1` joint position rad, `command.max_effort`는 N 기준 |
| Topic | `/joint_states` | `sensor_msgs/msg/JointState` | publish | joint_state_broadcaster/controller 기본 QoS; position rad, velocity rad/s |

CLI 실행 예:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
"{command: {position: 0.5, max_effort: 100.0}}"
```

현재 `rh_p12_rn_a_bringup/config/hardware_controller_manager.yaml` 기준:

```text
controller_manager.update_rate: 100 Hz
joint_state_broadcaster.type: joint_state_broadcaster/JointStateBroadcaster
gripper_controller.type: position_controllers/GripperActionController
gripper_controller.joint: rh_r1
gripper_controller.allow_stalling: true
```

현재 ROS 2 launch:

| launch | arguments | 실행 노드 |
| --- | --- | --- |
| `rh_p12_rn_a_bringup/launch/rh_p12_rn_a.launch.py` | `start_rviz=false`, `prefix=""`, `use_sim=false`, `use_fake_hardware=false`, `fake_sensor_commands=false`, `port_name=/dev/ttyUSB0` | `controller_manager/ros2_control_node`, `controller_manager/spawner`, `robot_state_publisher`, `rviz2` |
| `rh_p12_rn_a_bringup/launch/rh_p12_rn_a_gazebo.launch.py` | `world=empty_world` | `robot_state_publisher`, `ros_gz_sim/create`, `joint_state_broadcaster` spawner, `gripper_controller` spawner, `ros_gz_bridge/parameter_bridge` |
| `rh_p12_rn_a_moveit_config/launch/rh_p12_rn_a_moveit.launch.py` | `start_rviz=true`, `use_sim=false`, `warehouse_sqlite_path` launch 기본값, `publish_robot_description_semantic=true` | `moveit_ros_move_group/move_group`, `rviz2` |

현재 ROS 2 joint limit:

| 출처 | joint | lower | upper | max velocity | max acceleration | effort |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `rh_p12_rn_a_description/urdf/rh_p12_rn_a.xacro` | `rh_r1` | -0.001 rad | 1.101 rad | 6.5 rad/s | 미기재 | 1000 N 또는 N·m |
| `rh_p12_rn_a_description/urdf/rh_p12_rn_a.xacro` | `rh_r2` | -0.001 rad | 1.101 rad | 6.5 rad/s | 미기재 | 1000 N 또는 N·m |
| `rh_p12_rn_a_description/urdf/rh_p12_rn_a.xacro` | `rh_l1` | -0.001 rad | 1.101 rad | 6.5 rad/s | 미기재 | 1000 N 또는 N·m |
| `rh_p12_rn_a_description/urdf/rh_p12_rn_a.xacro` | `rh_l2` | -0.001 rad | 1.101 rad | 6.5 rad/s | 미기재 | 1000 N 또는 N·m |
| `rh_p12_rn_a_moveit_config/config/joint_limits.yaml` | `rh_r1`, `rh_r2`, `rh_l1`, `rh_l2` | 미기재 | 미기재 | 6.5 rad/s | 6.5 rad/s2 | 미기재 |

현재 ROS 2 예외/안전 규칙:

- launch 파일 자체에는 force 초과, timeout, 자동 정지 로직이 없다.
- `GripperActionController`의 `allow_stalling`은 true다.
- MoveIt 설정은 `gripper_controller` 이름, `type: GripperCommand`, `action_ns: gripper_cmd`, joints=`rh_r1`만 명시한다.
- 실제 gripper controller가 떠 있는지는 `ros2 action list`, `ros2 control list_controllers`로 확인해야 한다.

## 원본 포팅용 인터페이스 계약

이 섹션은 `RH-P12-RN-A`의 현재 gripper action 구성이 아니라, ROS 1 원본 기능을 별도 ROS 2 driver/backend로 옮길 때 참고할 인터페이스 기준이다. 기존 ROS 1 호환이 필요하면 별도 bridge/compat node에서 원본 이름을 매핑한다.

### 포팅용 토픽

| 토픽 | 타입 | 방향 | 의미 |
| --- | --- | --- | --- |
| `/rh_p12_rn/command/goal_position` | `std_msgs/msg/Int32` 또는 custom command msg | subscribe | `gripper` raw goal position |
| `/rh_p12_rn/command/goal_current` | `std_msgs/msg/Int32` | subscribe | `gripper` raw goal current |
| `/rh_p12_rn/command/torque_enable` | `std_msgs/msg/Bool` | subscribe | torque on/off |
| `/rh_p12_rn/state/present_position` | `std_msgs/msg/Int32` | publish | 현재 raw position |
| `/rh_p12_rn/state/present_current` | `std_msgs/msg/Int32` | publish | 현재 raw current |
| `/rh_p12_rn/joint_states_expanded` | `sensor_msgs/msg/JointState` | publish | RViz용 확장 joint state |
| `/rh_p12_rn/rh_p12_rn_position/command` | `std_msgs/msg/Float64` | subscribe | 시뮬레이션 단일 구동 joint command |
| `/rh_p12_rn/rh_r2_position/command` | `std_msgs/msg/Float64` | publish | 시뮬레이션 보조 joint command |
| `/rh_p12_rn/rh_l1_position/command` | `std_msgs/msg/Float64` | publish | 시뮬레이션 보조 joint command |
| `/rh_p12_rn/rh_l2_position/command` | `std_msgs/msg/Float64` | publish | 시뮬레이션 보조 joint command |

### 포팅용 서비스

원본의 `GetItemValue.srv`는 유지 가치가 있다.

| 서비스 | 타입 | 방향 | 의미 |
| --- | --- | --- | --- |
| `/rh_p12_rn/get_item_value` | `rh_p12_rn_interfaces/srv/GetItemValue` 또는 원본 호환 `GetItemValue` | server | `gripper`의 상태/control table 값 조회 |

```text
string joint_name
string item_name
---
uint32 value
bool success
string message
```

원본 서비스에는 `success`, `message`가 없다. ROS 2 재작성에서 실패 사유가 필요하면 두 필드를 추가하고, 원본 완전 호환이 목적이면 아래 원본 형태를 그대로 사용한다.

```text
string joint_name
string item_name
---
uint32 value
```

지원해야 하는 `item_name`:

| item_name | 의미 |
| --- | --- |
| `torque_enable` | torque on/off 상태 |
| `goal_position` | 목표 position raw value |
| `goal_velocity` | 목표 velocity raw value |
| `goal_current` | 목표 current raw value |
| `goal_acceleration` | 목표 acceleration raw value |
| `is_moving` | 움직임 여부 |
| `present_position` | 현재 position raw value |
| `present_velocity` | 현재 velocity raw value |
| `present_current` | 현재 current raw value |

요청 규칙:

- `joint_name`은 원본 기준 `gripper`만 유효하다.
- 지원하지 않는 `item_name`은 실패 처리해야 한다.
- 배열 인덱스를 직접 접근하기 전에 길이를 검사해야 한다.

### 원본 포팅용 action 정책

원본에는 action이 없다. 원본 호환 driver를 새로 만들 때는 아래 요구가 없다면 action을 추가하지 않는다. 현재 `RH-P12-RN-A`의 `/gripper_controller/gripper_cmd`는 ros2_control gripper controller가 제공하는 표준 action이므로 이 원칙의 예외다.

- 장시간 grasp sequence를 goal/cancel/result로 관리해야 하는 경우
- MoveIt gripper command action과 통합해야 하는 경우
- 상위 시스템이 action interface를 요구하는 경우

## 원본 ROS 1 인터페이스 매핑

원본 코드나 기존 문서를 참조할 때 ROS 1 이름은 아래처럼 해석한다.

| 원본 ROS 1 인터페이스 | 타입 | 의미 | ROS 2 구현 시 처리 |
| --- | --- | --- | --- |
| `/robotis/direct/sync_write_item` | `robotis_controller_msgs/SyncWriteItem` | `goal_position` 직접 위치 명령 | 위치 명령 API로 재구성 |
| `/robotis/sync_write_item` | `robotis_controller_msgs/SyncWriteItem` | torque/mode/current/velocity/acceleration 쓰기 | 제어 테이블 write API로 재구성 |
| `/robotis/rh_p12_rn_base/get_item_value` | `rh_p12_rn_base_module_msgs/GetItemValue` | 상태/control table 값 조회 | ROS 2 service로 재구성 |
| `/robotis/rh_p12_rn_base/present_position` | `std_msgs/Int32` | 현재 position publish | state topic으로 재구성 |
| `/robotis/rh_p12_rn_base/present_current` | `std_msgs/Int32` | 현재 current publish | state topic으로 재구성 |
| `/robotis/present_joint_states` | `sensor_msgs/JointState` | 원본 present joint state | expanded joint state 입력으로 사용 가능 |
| `/robotis/goal_joint_states` | `sensor_msgs/JointState` | 원본 goal joint state | expanded joint state 입력으로 사용 가능 |
| `/rh_p12_rn/rh_p12_rn_position/command` | `std_msgs/Float64` | Gazebo 단일 구동 joint command | ros2_control command topic 또는 변환 노드 입력 |

## 원본 메시지 구조

### `GetItemValue.srv`

```text
string joint_name
string item_name
---
uint32 value
```

### `WriteCtrlItem.msg`

```text
string joint_name
string item_name
uint32 value
```

`WriteCtrlItem.msg`는 원본에서 실제 사용처가 없다. ROS 2에서는 아래 중 하나를 선택한다.

| 선택 | 기준 |
| --- | --- |
| 만들지 않음 | 원본 사용처까지 정확히 재현할 필요가 없을 때 |
| `SetItemValue.srv`로 대체 | 요청-응답 성공 여부가 필요한 경우 |
| command topic용 msg로 재설계 | 여러 item write를 topic으로 계속 흘려야 하는 경우 |

원본에서 실제 쓰기 명령에 사용한 외부 메시지는 `robotis_controller_msgs/SyncWriteItem`이다. 코드 사용 형태는 다음 필드만 전제한다.

| 필드 | 사용값 |
| --- | --- |
| `joint_name` | `["gripper"]` |
| `item_name` | `torque_enable`, `operating_mode`, `goal_current`, `goal_velocity`, `goal_acceleration`, `goal_position` |
| `value` | raw integer array |

## 상태 및 제어 항목

ROS 2 driver/backend가 내부 상태로 관리해야 하는 항목:

| 항목 | 타입 기준 | 설명 |
| --- | --- | --- |
| `torque_enable` | integer/bool | 원본은 bulk read table의 uint 값 |
| `goal_position` | uint/int | 목표 position raw value |
| `goal_velocity` | uint/int | 목표 velocity raw value |
| `goal_current` | int 가능 | current mode에서 음수 값도 GUI가 사용 |
| `goal_acceleration` | uint/int | 목표 acceleration raw value |
| `is_moving` | bool/int | 자동 반복 판단에 사용 |
| `present_position` | uint/int | 현재 position raw value |
| `present_velocity` | uint/int | 현재 velocity raw value |
| `present_current` | int | 현재 current raw value |

원본 GUI 범위:

| 항목 | 범위 |
| --- | --- |
| `goal_current` | 기본 `0..820`, current mode에서는 `-820..820` |
| `goal_velocity` | `0..1023` |
| `goal_acceleration` | `0..1023` |
| `goal_position` | UI 범위 `0..1150`; 버튼 close 값은 `740` |

원본 mode 값:

| mode | 값 | 의미 |
| --- | --- | --- |
| current mode | `0` | current control |
| current-based position mode | `5` | position command와 current limit을 함께 사용 |

## 실제 장비 제어 기준

실제 장비를 제어할 때의 기준:

1. 장치 이름은 원본 기준 `gripper`다.
2. 기본 포트는 `/dev/ttyUSB0`, baudrate는 `2000000`, Dynamixel ID는 `1`, protocol은 `2.0`이다.
3. 제어 주기는 원본 설정상 `8 ms`다.
4. 초기 설정은 `return_delay_time=1`이다.
5. 위치 명령은 raw `goal_position`을 backend가 요구하는 단위로 변환해 전달한다. 원본은 `convertValue2Radian(value)`를 사용했다.
6. 상태 조회 서비스는 위 `item_name` 목록만 지원한다.
7. publish 주기는 구현 환경에 맞게 정하되, 원본은 ROBOTIS control cycle에서 `present_position`, `present_current`를 계속 발행했다.
8. 잘못된 `joint_name`, 빈 배열, 지원하지 않는 item은 무시하지 말고 명시적으로 실패 처리한다.

## RViz joint 변환 계약

원본 URDF는 4개의 revolute joint를 가진다.

| joint | limit |
| --- | --- |
| `rh_p12_rn` | `0.0..1.1` |
| `rh_r2` | `0.0..1.0` |
| `rh_l1` | `0.0..1.1` |
| `rh_l2` | `0.0..1.0` |

단일 입력 joint `rh_p12_rn`으로 전체 손가락 joint state를 만들 때 원본 변환식은 아래와 같다.

| 출력 joint | position |
| --- | --- |
| `rh_p12_rn` | 입력값 그대로 |
| `rh_r2` | `rh_p12_rn * (1.0 / 1.1)` |
| `rh_l1` | `rh_p12_rn` |
| `rh_l2` | `rh_p12_rn * (1.0 / 1.1)` |

ROS 2에서는 `sensor_msgs/msg/JointState`를 사용한다. 입력 JointState에 이미 다른 joint가 있으면 원본처럼 기존 name/position을 유지하고, `rh_p12_rn`을 발견했을 때 위 3개 joint를 추가한다.

## Gazebo/시뮬레이션 변환 계약

원본 Gazebo는 `/rh_p12_rn` namespace에서 effort position controller 4개를 사용한다.

| controller | joint | command |
| --- | --- | --- |
| `rh_p12_rn_position` | `rh_p12_rn` | 단일 입력 |
| `rh_r2_position` | `rh_r2` | 변환 출력 |
| `rh_l1_position` | `rh_l1` | 변환 출력 |
| `rh_l2_position` | `rh_l2` | 변환 출력 |

단일 입력 command에서 보조 joint command를 만들 때 원본 변환식은 아래와 같다.

| 출력 command | 값 |
| --- | --- |
| `rh_r2_position` | `input * (1.0 / 1.1)` |
| `rh_l1_position` | `input` |
| `rh_l2_position` | `input * (1.0 / 1.1)` |

ROS 2 Gazebo/ros2_control에서는 controller 종류가 달라질 수 있지만 이 변환식을 기준으로 한다.

## URDF/ros2_control 기준

원본 `rh_p12_rn.xacro`의 구조:

| joint | type | parent -> child |
| --- | --- | --- |
| `world_fixed` | fixed | `world` -> `rh_p12_rn_base` |
| `rh_p12_rn` | revolute | `rh_p12_rn_base` -> `rh_p12_rn_r1` |
| `rh_r2` | revolute | `rh_p12_rn_r1` -> `rh_p12_rn_r2` |
| `rh_l1` | revolute | `rh_p12_rn_base` -> `rh_p12_rn_l1` |
| `rh_l2` | revolute | `rh_p12_rn_l1` -> `rh_p12_rn_l2` |

원본 transmission은 모두 `EffortJointInterface`다. ROS 2에서는 `ros2_control`의 `command_interfaces`와 `state_interfaces` 정의를 확인한다. position command 기반 구성은 원본 PID effort controller와 동작 차이가 있을 수 있다.

## 원본 launch에서 포팅 시 주의할 점

아래는 ROS 2로 그대로 옮기면 안 되는 원본 문제다.

| 항목 | 원본 상태 | ROS 2 처리 |
| --- | --- | --- |
| `rh_p12_rn_ctrl.launch` xacro | `thormang3.xacro` 참조, 저장소에 없음 | `rh_p12_rn.xacro` 기준으로 새 launch 작성 |
| `offset.yaml` | launch가 참조하지만 저장소에 없음 | offset 기능이 필요할 때만 파라미터화 |
| `rh_p12_rn_gui/package.xml` | `</url>>` 오타 | ROS 2 package.xml 작성 시 반영하지 않음 |
| `BaseModule::setPosition` | 배열 길이 검사 없이 `joint_name[0]`, `value[0]` 접근 | 입력 배열 길이 확인 필요 |
| `BaseModule::~BaseModule` | thread join을 무조건 호출 | ROS 2 lifecycle/shutdown 안전 처리 |

## RH-P12-RN-A 실행 인터페이스 원문 계약

이 절은 현재 ROS 2 패키지 `RH-P12-RN-A`에서 실제 실행되는 gripper controller, MoveIt controller mapping, ros2_control hardware interface만 정리한다.

### 실행 action payload

- Endpoint: `/gripper_controller/gripper_cmd`
- Type: `control_msgs/action/GripperCommand`
- QoS: ROS 2 action 기본 QoS. goal/result/cancel service는 Reliable/Volatile, feedback/status topic은 action 기본 profile.
- Goal 구조:

```yaml
command:
  position: float64   # rad. `rh_r1` 명령 위치. URDF limit: -0.25 ~ 1.1 rad.
  max_effort: float64 # N. 0.0이면 controller 기본 처리에 맡김.
```

- Result 구조:

```yaml
position: float64 # rad
effort: float64   # N
stalled: bool
reached_goal: bool
```

- Feedback 구조:

```yaml
position: float64 # rad
effort: float64   # N
stalled: bool
reached_goal: bool
```

### MoveIt controller mapping

| 항목 | 값 |
|---|---|
| plugin | `moveit_simple_controller_manager/MoveItSimpleControllerManager` |
| controller_names | `gripper_controller` |
| controller type | `GripperCommand` |
| action_ns | `gripper_cmd` |
| joint | `rh_r1` |

### ros2_control command/state interface

| joint | command_interface | state_interface | 단위/주의사항 |
|---|---|---|---|
| `rh_r1` | `position` | `position`, `velocity`, `effort` | position rad, velocity rad/s, effort N. `gripper_controller`가 명령하는 유일한 active joint. |
| `rh_r2` | 없음 | `position`, `velocity`, `effort`, `torque_enable`, `hardware_state` | mimic/passive state. `rh_r1` 명령에서 URDF mimic 식으로 파생. |
| `rh_l1` | 없음 | `position`, `velocity`, `effort`, `torque_enable`, `hardware_state` | mimic/passive state. |
| `rh_l2` | 없음 | `position`, `velocity`, `effort`, `torque_enable`, `hardware_state` | mimic/passive state. |
| `DynamixelHardware` custom interface | `Goal Position` | `Present Position`, `Present Velocity`, `Present Current` | Dynamixel SDK register-level interface 이름. ROS 표준 joint command API로 직접 쓰지 않는다. |

### hardware plugin parameter

| parameter | 값 |
|---|---|
| `usb_port` | launch arg `port_name`, 기본 `/dev/ttyUSB0` |
| `baud_rate` | `57600` |
| `number_of_joints` | `1` |
| `joint_ids` | `1` |
| `joint_model_numbers` | `1060` |
| `joint_names` | `rh_r1` |
| `dynamixel_state_pub_msg_name` | `dynamixel_hardware_interface/dxl_state` |
| `get_dynamixel_data_srv_name` | `dynamixel_hardware_interface/get_dxl_data` |
| `set_dynamixel_data_srv_name` | `dynamixel_hardware_interface/set_dxl_data` |
| `reboot_dxl_srv_name` | `dynamixel_hardware_interface/reboot_dxl` |
| `set_dxl_torque_srv_name` | `dynamixel_hardware_interface/set_dxl_torque` |

### mimic 변환식

| joint | multiplier | offset | 식 |
|---|---:|---:|---|
| `rh_r2` | `-1` | `0` | `rh_r2 = -rh_r1` |
| `rh_l1` | `-1` | `0` | `rh_l1 = -rh_r1` |
| `rh_l2` | `1` | `0` | `rh_l2 = rh_r1` |
