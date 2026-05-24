# [System Architecture] ROS2 기반 VR 로봇 제어 시스템 (Sim-to-Real)

이 문서는 전체 프로젝트의 기준 아키텍처, 실제 구현 상태, 설계 원칙을 정의하는 마스터 문서입니다.  
`Current.md`는 현재 사실, `PLAN.md`는 실행 순서, 이 문서는 장기 구조 기준을 담당합니다.

---

## 1. 시스템 핵심 목표
**"Meta Quest 2(VR)를 활용하여 원격지에서 Doosan E0509와 RH-P12-RN-A를 직관적이고 안전하게 조종하고, 시뮬레이션 검증을 거쳐 실제 하드웨어 제어로 전환한다."**

단, 2026-05-16 기준으로 **Sim-to-Real은 목표 아키텍처이지 아직 완성 상태는 아닙니다.**  
현재는 공통 로봇 모델과 Mock `ros2_control` 정의를 분리해 경계만 정리한 상태이며, 실제 하드웨어 백엔드는 장비 확보 후 구현해야 합니다.

---

## 2. 현재 시스템 경계

### 2-1. 우리 프로젝트가 직접 관리하는 패키지
* `vrobot_description`
* `vrobot_moveit_config`
* `vrobot_command`

### 2-2. 외부 의존 패키지
* `doosan-robot2`
* `RH-P12-RN-A`
* `ros_tcp_endpoint`

현재 `src`는 단일 Git 저장소가 아니라 여러 저장소가 나란히 들어간 워크스페이스 구조입니다.  
따라서 우리 프로젝트의 변경과 외부 패키지 변경을 분리해 관리해야 하며, 외부 저장소 내부 수정은 마지막 수단으로만 사용합니다.

---

## 3. 하드웨어 및 소프트웨어 스택

### 3-1. ROS2 서버
* Ubuntu 22.04 / ROS2 Humble
* Motion Planning: `MoveIt2`
* Planning Pipeline: 현재 실제 설정은 `OMPL` 중심
* Hardware Interface: `ros2_control`
* Controller: `JointTrajectoryController`
* TCP Bridge: `ros_tcp_endpoint`

### 3-2. Unity 클라이언트
* Windows 측 현재 상태: Unity `3D (Universal)` 설치 완료
* Unity 정확한 에디터 버전은 문서에 아직 확정 기록이 없어 확인 필요
* VR 플랫폼: Meta Quest 2
* 통신: `ROS-TCP-Connector`
* 모델 임포트: `URDF Importer`

---

## 4. 데이터 흐름

### 4-1. 현재 구현 완료된 흐름
1. `vrobot_full_sim.launch.py`가 `robot_state_publisher`, `ros2_control_node`, MoveIt2, RViz2를 함께 기동
2. `joint_state_broadcaster`가 관절 상태를 `/joint_states`로 퍼블리시
3. MoveIt2가 `doosan_arm_controller`, `gripper_controller`로 궤적 실행
4. `vrobot_vr_bridge.launch.py`가 별도 TCP 서버를 기동

### 4-2. 아직 구현되지 않은 목표 흐름
1. Unity가 VR 컨트롤러 입력을 ROS 좌표계로 변환
2. Unity가 목표 Pose와 그리퍼 명령을 제한된 빈도로 전송
3. ROS2의 신규 명령 처리 노드가 목표를 검증하고 MoveIt2에 전달
4. Planning Scene, IK 실패 처리, 상태 피드백, Haptic 알림이 연동
5. 실제 하드웨어 백엔드와 Mock 백엔드를 선택적으로 전환

---

## 5. 핵심 설계 원칙

### 5-1. Brain & Muscle Separation
MoveIt2와 하드웨어 백엔드는 분리되어야 합니다.  
이를 위해 최종 구조는 다음을 만족해야 합니다.
* 로봇 기구학/시맨틱 모델은 공통 사용
* `ros2_control` 하드웨어 플러그인은 xacro 인자 또는 별도 하드웨어 조각으로 교체 가능
* Mock용 런치와 Real용 런치는 같은 상위 인터페이스를 유지

현재는 공통 로봇 모델과 Mock 하드웨어 정의를 분리했으며, Real backend만 장비 확보 후 추가하면 되는 상태입니다.

### 5-2. One Source of Truth
다음 항목은 문서와 코드에서 중복 정의를 줄여야 합니다.
* 실제 사용 중인 플래너
* Unity 버전
* 컨트롤러 실행 방식
* 안전 제한 값

한 문서가 바뀌면 다른 문서가 자동으로 오래된 상태가 되지 않도록,  
`System.md`는 원칙, `Current.md`는 사실, `PLAN.md`는 다음 작업만 기록합니다.

### 5-3. Fail-Safe First
안전 기능은 VR 조작성보다 먼저 들어가야 합니다.
* 작업공간 제한
* 통신 타임아웃
* Planning Scene 충돌 객체
* 명령 거부/보류 정책
* 실제 하드웨어 전환 전 저속 검증

---

## 6. 구조 점검 결과와 권장 방향

| 항목 | 현재 상태 | 구조 판단 | 권장 방향 |
| --- | --- | --- | --- |
| 하드웨어 백엔드 | Mock `ros2_control` 정의를 별도 xacro로 분리 완료 | Real 백엔드는 아직 없음 | 실제 장비 확보 후 같은 계약으로 Real 정의 추가 |
| VR 명령 계층 | `vrobot_command` 패키지와 검증 전용 `vr_command_handler` 1차 구현 완료 | 아직 planning/execution 연결 전 | 입력 검증 -> 상태 정규화 -> 계획/실행 순으로 확장 |
| 플래너 선언 | 런치에서 `ompl`, `chomp` 모두 선언 | 실제 설정은 OMPL만 실질 사용 | CHOMP를 쓸 계획이 없다면 선언 제거, 쓸 계획이면 설정 추가 |
| 런치 파일 | `vrobot_full_sim.launch.py`는 순차 기동, `vrobot_mock_hw.launch.py`는 병렬 기동 | 동일 역할의 런치 품질이 다름 | 기준 런치를 하나로 정하고 나머지는 정리 또는 동일 수준으로 맞춤 |
| 안전 계층 | 정적 바닥 collision object 1차 적용 완료 | workspace 제한, watchdog, 명령 핸들러는 미구현 | 환경 충돌 -> 입력 제한 -> 통신 감시 순서로 확장 |
| 패키지 메타데이터 | `vrobot_description/package.xml` 1차 보강 완료 | 일부 경계 문제는 남음 | 현재 실행 의존성은 유지하고 통합 런치 책임을 별도 정리 |
| 패키지 경계 | 통합 시뮬레이션 런치가 `vrobot_description` 안에서 `vrobot_moveit_config`를 참조 | 양방향 의존성 위험 | 장기적으로 별도 bringup 패키지로 통합 런치 이동 검토 |
| 문서 체계 | 일부 문서 간 버전/상태 불일치 | 의사결정 오류 가능 | 상태 문서와 계획 문서를 주기적으로 동기화 |

---

## 7. 유기적 해결 원칙

문제를 하나씩 해결하되, 다음 순서를 지켜 다른 문제를 만들지 않습니다.

1. **하드웨어 추상화 정리**
   - 먼저 Mock/Real 전환 구조를 분리
   - 이후 안전 로직과 명령 로직을 공통 상위 계층에 배치
2. **안전 계층 선행**
   - Planning Scene, workspace 제한, watchdog을 먼저 정의
   - 그 뒤 Unity 제어 입력을 연결
3. **제어 모드 분리**
   - 초기에는 Way-point 제어만 지원
   - 실시간 추종은 별도 Servo 아키텍처로 확장
4. **문서와 코드 동기화**
   - 구현 전 상태를 완료로 쓰지 않음
   - 문서의 "목표"와 "현재"를 분리

이 순서를 따르면 통신 기능을 추가하면서 안전성이 빠지는 문제, 실기 전환을 준비하면서 시뮬레이션 안정성을 깨는 문제를 피할 수 있습니다.

---

## 8. 안전 정책 책임 분리

### 8-1. MoveIt / Planning Scene 책임
* 로봇과 외부 환경의 기하학적 충돌 판정
* 바닥, 책상, 고정 장애물처럼 공간에 존재하는 물체 관리
* 경로 전체가 충돌하지 않는지 검증

### 8-2. 명령 처리 계층 책임
* Unity에서 들어온 목표 Pose의 형식, frame, 시간 유효성 검증
* 작업공간 범위 밖 명령의 조기 거부
* IK 실패, planning 실패, timeout을 공통 상태 코드로 변환
* 실패한 명령을 실행하지 않고 Unity에 피드백

현재 구현 상태:
* 별도 패키지 `vrobot_command` 생성
* `/vr/pose_goal` 입력 수신
* `/vr/command_status` 출력 발행
* frame 검증과 파라미터 기반 workspace 검증 완료
* plan-only 연결 완료
* `execute_enabled` 파라미터 기반 선택적 execution 연결 완료
* planning / execution timeout과 의미 기반 실패 상태 정규화 1차 완료

### 8-3. Unity 클라이언트 책임
* 사용자가 명백히 잘못된 범위를 가리키지 않도록 1차 가이드
* 전송률 제한
* 서버가 보낸 상태 코드를 UI/Haptic으로 표현

### 8-4. 설계 원칙
* **Unity만 믿지 않는다.** 클라이언트 검증은 UX 최적화이고, 최종 거부권은 ROS2가 가진다.
* **Planning Scene만 믿지 않는다.** 충돌 경로 검사는 해주지만, 명령 폭주와 잘못된 입력 의미까지 설명해주지는 않는다.
* **workspace 수치는 코드에 하드코딩하지 않는다.** 실제 작업 셀과 실기 조건이 확정되면 파라미터로 조정한다.
