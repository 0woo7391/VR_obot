# 🚀 Vrobot 시스템 실행 매뉴얼 (Run Guide)

이 문서는 Ubuntu 환경에서 ROS2 로봇 시스템과 통신 브리지를 가동하는 모든 실행 명령어와 순서를 한눈에 볼 수 있도록 정리한 치트시트(Cheat Sheet)입니다.

---

## 0. 필수 사전 작업 (환경 변수 로드)
새 터미널을 열 때마다 반드시 작업 공간(Workspace)의 환경 변수를 불러와야 합니다.
```bash
cd ~/Vrobot_ws
sr  
# (또는 source install/setup.bash)
```

---

## 1. 기본 모드: 로봇 시뮬레이션 단독 실행
Unity 없이, Ubuntu 내에서 RViz2를 켜서 로봇의 플래닝과 구동(Execute)을 테스트할 때 사용합니다.

*   **터미널 1개만 사용:**
    ```bash
    ros2 launch vrobot_description vrobot_full_sim.launch.py
    ```
*   **실행되는 기능:** 가상 로봇 생성 + 제어기(Controllers) + MoveIt2(두뇌) + RViz2 시각화

---

## 2. 연동 모드: 전체 시스템 동시 가동
Windows의 Unity와 연동하여 로봇을 원격 조종할 때 사용합니다. **반드시 2개의 터미널**을 띄워야 합니다.

*   **[터미널 1] 로봇 본체 가동 (두뇌와 근육)**
    ```bash
    ros2 launch vrobot_description vrobot_full_sim.launch.py
    ```
*   **[터미널 2] 통신 브리지 및 제어 핸들러 가동 (무전기 + 제어기)**
    ```bash
    ros2 launch vrobot_command unity_control.launch.py execute_enabled:=true
    ```
    *(실행 후 `Ready for pose goals` 문구가 뜨면 Unity 접속 대기 상태가 됩니다.)*


### Unity 연결 전 브리지 확인
브리지는 Unity보다 먼저 켭니다.

1. Ubuntu에서 위 unity_control launch 실행
2. 아래 로그 확인
   ```text
   Starting server on 0.0.0.0:10000
   ```
3. 현재 Unity 연결 값
   ```text
   ROS IP Address: 192.168.23.130
   ROS Port: 10000
   Protocol: ROS2
   ```
4. 필요하면 Ubuntu에서 포트 listen 확인
   ```bash
   ss -ltnp | grep 10000
   ```
5. Unity가 연결되면 bridge 터미널에 접속 로그가 추가됩니다.

---

## 3. 🖥️ 통합 제어 대시보드 (VRobot Dashboard) - 추천 방식
여러 터미널을 띄우고 명령어를 번거롭게 타이핑할 필요 없이, GUI 기반 단일 대시보드에서 시스템 기동, 프로세스 모니터링, 예외 진단을 일괄 수행할 수 있습니다.

*   **대시보드 실행 명령어:**
    ```bash
    python3 src/vrobot_command/scripts/vrobot_dashboard.py
    ```

### 💡 주요 기능 및 조작법
1.  **🚀 START INTEGRATION (원클릭 자동 시퀀서):**
    *   버튼을 클릭하면 **1단계(MoveIt2 시뮬레이션)**를 먼저 기동합니다.
    *   DDS 네트워크 상에 `move_group` 노드 기동이 감지되는 즉시, 자동으로 **2단계(Unity Mediator 브리지, `execute_enabled:=true`)**를 연이어 스폰하고 시퀀스를 마무리합니다.
2.  **💥 Force Kill All (안전한 정밀 청소):**
    *   시스템 비정상 종료 시 발생하는 10000번 포트 점유 충돌(`Address already in use`) 및 FastDDS 공유 메모리 파일 락(`/dev/shm/fastrtps_port*`)을 한 번에 강제 해제합니다.
    *   대시보드 본인 프로세스는 안전하게 살려둔 채로 백그라운드 찌꺼기 노드들만 조준 사살합니다.
3.  **DDS 노드 & 세션 감시판:**
    *   `move_group`, `unity_endpoint`, `vr_command_handler` 노드들의 생존 여부를 실시간으로 **🟢/🔴 상태등**으로 표기합니다.
    *   특히 **`unity_runtime (Play Session)`** 지표는 Windows Unity Editor에서 **실제 Play 버튼을 눌러 소켓 활성 세션이 수립될 때만 초록 불**로 연동되어 활성화 상태를 진단합니다.
4.  **4단계 모션 인디케이터:**
    *   유니티 타겟 조작에 따른 경로 계획 및 주행 제어 상태를 **`IDLE` ➔ `PLAN` ➔ `EXECUTE` ➔ `FAIL`**의 4단계 네온 바를 통해 직관적으로 표기합니다.

---

## 4. 🚨 시스템 수동 강제 종료 및 소켓 정화
대시보드를 쓰지 않고 터미널에서 수동 종료할 때, 10000번 소켓과 FastDDS 공유 메모리가 잠겨서 다음 구동 시 통신이 끊어지거나 노드 디스커버리가 정체되는 경우가 있습니다. 이 경우 아래 명령어로 깨끗하게 정화할 수 있습니다.

*   **수동 리소스 정밀 정화 명령어:**
    ```bash
    fuser -k 10000/tcp; killall -9 ros2 rviz2 robot_state_publisher ros2_control_node move_group; rm -f /dev/shm/fastrtps_port*
    ```

---

## 💡 개발 시 참고 팁
*   코드를 수정했거나 패키지를 새로 설치한 경우, 반드시 `cb` (또는 `colcon build`) 명령어로 빌드를 진행한 후, 터미널을 껐다 켜거나 `sr` 명령어로 환경 변수를 갱신해야 적용됩니다.
*   현재 Ubuntu 시스템의 통신 접속용 IP 주소는 **`192.168.23.130` (포트: 10000)** 입니다. Windows Unity 세팅 시 참고하세요.
