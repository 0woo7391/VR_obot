# [Project Current State] ROS2 기반 VR 로봇 제어 시스템 인계서 (v1.6 연동 환경 복구 및 자유 제어 테스트 반영)

## 1. 프로젝트 개요
* **목표:** Meta Quest 2를 이용해 Doosan E0509와 RH-P12-RN-A를 통합 제어
* **현재 단계:** ROS2 Mock Hardware + MoveIt2 시뮬레이션 검증 완료, Unity `/joint_states` 단방향 동기화 1차 검증 완료, Unity target 포즈 2터미널 송수신 복구 완료.
* **작업 환경:** Ubuntu 22.04, ROS2 Humble, Workspace `~/Vrobot_ws`

## 2. 현재 실제로 존재하는 핵심 패키지
* **직접 관리 패키지**
  - `vrobot_description`
  - `vrobot_moveit_config`
  - `vrobot_command`
* **외부 의존 패키지**
  - `doosan-robot2`
  - `RH-P12-RN-A`
  - `ros_tcp_endpoint`

`src` 루트 자체는 단일 Git 저장소가 아니며, 외부 패키지들은 각자 별도 저장소입니다.

## 3. 코드 기준 완료된 항목
* `vrobot.urdf.xacro`
  - E0509 `link_6`와 그리퍼 `rh_p12_rn_base`를 fixed joint로 결합
  - 공통 로봇 모델 역할 유지
  - `use_fake_hardware` 인자로 mock/real hardware 분기 구조 추가
* `vrobot_mock_ros2_control.xacro`
  - Mock `ros2_control` 정의를 별도 파일로 분리
* `ros2_controllers.yaml`
  - `doosan_arm_controller`
  - `gripper_controller`
  - `joint_state_broadcaster`
* `vrobot_moveit_config`
  - `doosan_arm`, `gripper` planning group 구성
  - KDL kinematics 설정
  - SRDF self-collision matrix 존재
  - `joint_limits.yaml`에 팔/그리퍼 제한값과 가속도 제한 정의
  - `ompl_planning.yaml`에 `RRTstar`와 Time Parameterization 설정
* `unity_control.launch.py` (vrobot_command 패키지)
  - `ros_tcp_endpoint`와 `vr_command_handler.py`를 동시 기동하여 2터미널 VR 연동 표준 인프라로 구성.
* Unity ↔ ROS2 단방향 동기화 완료
  - `unity_scripts/JointStateSubscriber.cs`를 통하여 팔 6축(`joint_1~6` -> `link_1~6`) 및 그리퍼 4축(`rh_r1/l1/r2/l2` -> `rh_p12_rn_*`) 동기화 성공.
  - Unity ArticulationBody의 중력 처짐(Sagging) 현상을 방지하기 위해 스크립트 단에서 `disableGravityOnMappedBodies=true` 및 `configureDriveOnStart=true`를 통한 강성(Stiffness) 보정 조치 완료.
* `vr_command_handler.py` 통신 패치 및 그리퍼 제어기 검증 완료
  - rclpy의 가비지 컬렉션(GC)에 의해 구독기가 해제되는 현상을 방지하기 위해 구독 개체를 클래스 멤버 변수(`self.pose_goal_subscription`, `self.gripper_goal_subscription`)에 저장하여 유지하도록 패치 완료.
  - `/vr/gripper_goal` 토픽을 발행하여 `GRIPPER_ACCEPTED -> GRIPPER_EXECUTED` 전 단계 구동 검증 완료.

## 4. 문서와 코드가 일치하지 않던 부분 및 해결 내역
* 과거 문서에는 `Run_Guide.md`가 있다고 적혀 있었지만 현재 워크스페이스 상의 `Run_Guide.md` 지침이 과거 구버전 런치(`vrobot_vr_bridge.launch.py`)를 안내하고 있어, 현재 사양에 맞는 `unity_control.launch.py` 기준으로 정정 및 동기화 완료.
* 1터미널 마스터 런치 파일(`vrobot_vr_full.launch.py`) 기획은 동시 기동에 따른 컨트롤러 라이프사이클 붕괴 및 세그폴트 이슈로 인해 **영구 폐기**하고, 2터미널 정상 연동으로 회귀 완료.

## 5. 현재 구조상 주요 이슈
* **이슈 A. Unity ➔ ROS2 쿼터니언 변환 및 Y축 정렬 고정 현상:** Unity에서 구체를 드래그할 때 로봇 그리퍼 머리 자세가 항상 일정 방향(Y축 정렬)으로만 묶이는 자세 제약 문제를 파헤쳐야 함.
* **이슈 B. 자유 동작을 위한 Clamping 제거:** 사용자의 조작성 한계를 넓히기 위해 유니티 C# 스크립트 및 ROS2 YAML 설정의 Clamping 장치를 모두 철회하고 날것의 포즈 전송 방식으로 복원.

## 6. 현재 멈춘 지점
* `vr_command_handler`에서 frame 오류와 workspace 이탈 명령 거부 검증 완료
* `vr_command_handler`에서 plan-only 경로 `ACCEPTED -> PLANNED` 검증 완료
* `vr_command_handler`에서 선택적 execute 경로 `ACCEPTED -> PLANNED -> EXECUTED` 검증 완료
* `/vr/gripper_goal` 닫기 `0.8`, 열기 `0.0` 테스트에서 `GRIPPER_ACCEPTED -> GRIPPER_EXECUTED`와 `/joint_states` gripper joint 값 반영 확인 완료
* TCP 엔드포인트는 `0.0.0.0:10000`에서 기동 확인 완료
* Ubuntu LAN IP `192.168.23.130` 확인 완료
* Windows Unity에서 ROS-TCP-Connector 연결과 `/joint_states` 단방향 동기화 1차 검증 완료
* RViz/MoveIt Execute 시 Unity 팔 관절 추종 확인 완료
* **[진행 예정]** Unity targetTransform 회전 전송 쿼터니언 정합성 디버깅 진행 대기 중.

## 7. 다음 액션
1. **Unity 조종 구체 Y축 회전 고정 디버깅 및 좌표 변환식 검증**
2. **Unity gripper open/close 시각화 매핑 안정화**
3. **mock 기준 end-to-end 동작 확인 및 실기 전환 준비**
