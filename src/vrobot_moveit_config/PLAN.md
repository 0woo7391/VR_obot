# [Phase 3] Unity VR ↔ ROS2 통합 연동 및 Sim-to-Real 실행 계획

이 문서는 현재 코드 상태를 기준으로, 기능을 추가하면서 다른 문제를 만들지 않도록 **의존성 순서가 있는 실행 계획**을 정의합니다.

---

## 1. 실행 원칙
1. 구현 전제조건이 충족되지 않은 기능은 뒤로 미룹니다.
2. 억지로 통합 기동 런치를 만들기보다, 각 노드가 제어기 매니저 라이프사이클에 맞춰 안정적으로 뜨는 표준 2터미널 가동 방식을 고수합니다.
3. Way-point 제어와 실시간 추종 제어를 섞지 않습니다.
4. 문서의 완료 상태는 실제 코드와 검증 결과가 있을 때만 갱신합니다.
5. 현재 구조가 다음 기능 구현을 막지 않는다면, 유지보수성 정리는 end-to-end 동작 확인 뒤에 수행합니다.
6. 우선순위는 "최종 작동 경로 확보"를 먼저 두고, 큰 구조 정리는 통합 검증 뒤 점검합니다.

---

## 2. 단계별 실행 계획

### Phase 3-0. 실기 전환 최소 기반 정리
**목표:** 다음 기능 구현을 막는 최소한의 Sim-to-Real 기반만 먼저 정리

1. Mock `ros2_control` 정의 분리 완료
2. `use_fake_hardware` 기반 hardware switching 구조 추가 완료
3. Real backend 추가 시 유지할 공통 controller/joint interface 계약 문서화
4. `vrobot_description/package.xml` 실행 의존성 1차 보강 완료
5. Unity 에디터 버전 기준 확정
6. 통합 런치 강제 도입은 기동 충돌 문제로 인해 기각하고 2터미널 표준 방식을 유지

**완료 기준**
* 문서와 코드에서 하드웨어 경계 상태가 일치
* 새 환경에서도 `rosdep` 기준 의존성 추적 가능
* Unity 버전 문서 불일치 제거
* mock/real 전환용 상위 스위치가 코드에 존재

### Phase 3-1. 안전 계층 선행 구현
**목표:** Unity 입력을 받기 전에 거부/정지/회피 기준 확보

1. Planning Scene에 바닥 collision object 추가 완료
2. 책상 collision object는 실제 치수 확정 후 추가
3. 안전 정책 책임 분리 확정
   - Planning Scene: 외부 환경 충돌 판정
   - `vr_command_handler`: 입력 검증, workspace 거부, 실패 상태 정규화
   - Unity: 1차 가이드, 전송률 제한, 피드백 표시
4. 작업공간 제한은 하드코딩이 아닌 파라미터 기반으로 설계
5. planning/execution timeout 구현 완료
6. IK 실패, 경로 생성 실패, timeout 발생 시 공통 상태 코드 1차 구현 완료
7. 통신 watchdog과 명령 만료 정책 1차 구현 완료

**완료 기준**
* 도달 불가 Pose가 실행되지 않음
* 바닥 충돌 경로가 차단됨
* 책상 치수 확정 후 책상 충돌 경로도 차단됨
* 통신 단절 시 새 명령을 더 받지 않고 안전 상태로 전환됨

### Phase 3-2. ROS2 명령 처리 계층 구현
**목표:** Unity가 붙기 전에도 ROS2 내부에서 검증 가능한 제어 API 확보

1. 별도 `vrobot_command` 패키지와 신규 노드 `vr_command_handler` 1차 구현 완료
2. 입력 토픽과 출력 상태 토픽 정의
   - 입력: Pose Goal, gripper command
   - 출력: accepted / rejected / planned / executed / failed 상태
3. frame 검증과 파라미터 기반 workspace 검증 완료
4. MoveIt plan-only 연동 완료
5. 선택적 execution 연동 완료 (`execute_enabled=false` 기본)
6. Way-point 기반 Pose Goal만 우선 지원
7. 그리퍼 명령 경로 추가 완료
8. 팔/그리퍼 명령은 독립 검증이 가능하도록 책임 분리 유지
9. 실패 상태를 Unity가 소비할 수 있는 형태로 publish

**완료 기준**
* ROS2 CLI 또는 테스트 publisher만으로 명령 수신, 거부, 실행, 실패 피드백 검증 가능

### Phase 3-3. 네트워크 및 브리지 검증
**목표:** Unity 없이도 연결 품질과 포트 구성을 분리 검증

1. Windows와 Ubuntu가 같은 네트워크 대역인지 확인
2. TCP 포트 `10000` Ubuntu 측 listen 확인 완료
3. `unity_control.launch.py`로 endpoint 및 핸들러 기동 완료
4. 브리지 연결과 ROS 토픽 송수신 지연 측정

**완료 기준**
* TCP 연결 성공
* `/joint_states` 전달 확인
* 지연 기준과 끊김 시나리오 측정값 확보

### Phase 3-4. Unity 디지털 트윈 단방향 동기화
**목표:** 제어 전에 시각화만 먼저 검증

1. `ROS-TCP-Connector`, `URDF Importer` 설치
2. 로봇 모델과 mesh 임포트 완료
   - 검증 기준 패키지: `unity_vrobot_y_axis_E_clean.zip`
   - import 설정: `Axis Type=Y Axis`, `Mesh Decomposer=Unity`
   - 팔/그리퍼 visual 및 collision 정상 표시 확인
3. `/joint_states` 구독
4. ROS 좌표계와 Unity 좌표계 변환 검증
5. PC GUI 또는 테스트 씬에서 관절 동기화 검증

**완료 기준**
* RViz와 Unity가 같은 관절 상태를 표현
* 관절 축, 회전 방향, 초기 자세가 일치

### Phase 3-5. Unity -> ROS2 Way-point 제어 및 좌표계 검증 (현재 활성 단계)
**목표:** 실시간 추종이 아닌 안정적인 점대점 제어 완성 및 자세 정합성 확보

1. 컨트롤러 입력을 Pose Goal로 변환
2. **[최우선 과제]** Unity 조종 구체의 Y축 회전 정렬 고정 원인 규명 및 쿼터니언 변환 정합성 검증
3. 10Hz 이하 또는 거리 임계값 기반으로 전송 제한
4. ROS2 상태 코드를 Unity UI/Haptic에 반영
5. 그리퍼 입력은 별도 채널로 제한 범위만 매핑

**완료 기준**
* 구체를 자유롭게 회전시켜도 타깃 그리퍼가 해당 회전 각도를 정상 추종
* 과도한 입력이 플래너 큐를 폭주시키지 않음
* 거부된 목표와 실행된 목표가 Unity에서 구분됨
* 팔과 그리퍼 명령이 서로 간섭하지 않음

### Phase 3-6. 실기 전환 준비
**목표:** 시뮬레이션과 실기를 같은 상위 제어 구조로 연결

1. Real hardware backend 연결
2. 실제 하드웨어 전용 저속 profile 준비
3. E-Stop, Hold, Home 정책 연결
4. Mock과 Real의 controller interface 일치 검증

**완료 기준**
* 상위 명령 API는 유지
* 하드웨어만 교체해도 제어 흐름이 동일
* 저속/정지 검증 후에만 사람 근처 운용

---

## 3. 우선순위

### 현재 진행 우선순위
1. **[현재 작업]** Unity 조종 구체의 Y축 회전 고정 버그 디버깅 및 변환 매핑식 교차 검증
2. Unity gripper open/close 시각화 매핑 안정화
3. mock 기준 end-to-end 동작 확인 및 2터미널 연동 완전성 검증
4. 실기 전환 준비
