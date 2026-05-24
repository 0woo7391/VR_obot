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
Unity나 VR 없이, Ubuntu 내에서 RViz2를 켜서 로봇의 플래닝과 구동(Execute)을 테스트할 때 사용합니다.

*   **터미널 1개만 사용:**
    ```bash
    ros2 launch vrobot_description vrobot_full_sim.launch.py
    ```
*   **실행되는 기능:** 가상 로봇 생성 + 제어기(Controllers) + MoveIt2(두뇌) + RViz2 시각화

---

## 2. VR 연동 모드: 전체 시스템 동시 가동
Windows의 Unity VR과 연동하여 로봇을 원격 조종할 때 사용합니다. **반드시 2개의 터미널**을 띄워야 합니다.

*   **[터미널 1] 로봇 본체 가동 (두뇌와 근육)**
    ```bash
    ros2 launch vrobot_description vrobot_full_sim.launch.py
    ```
*   **[터미널 2] 통신 브리지 및 제어 핸들러 가동 (무전기 + 제어기)**
    ```bash
    ros2 launch vrobot_command unity_control.launch.py execute_enabled:=true
    ```
    *(실행 후 `Ready for VR pose goals` 문구가 뜨면 Unity 접속 대기 상태가 됩니다.)*

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

## 3. 🚨 시스템 강제 종료 및 청소 (강력 추천)
Ctrl+C를 눌러서 껐는데도 백그라운드에 프로세스가 찌꺼기로 남아 충돌을 일으키는 경우, 새 런처를 켜기 전에 모든 프로세스를 강제로 깔끔하게 죽이는 명령어입니다.

*   **프로세스 초기화 (에러 무시):**
    ```bash
    killall -9 ros2 rviz2 robot_state_publisher ros2_control_node spawner move_group python3
    ```

---

## 💡 개발 시 참고 팁
*   코드를 수정했거나 패키지를 새로 설치한 경우, 반드시 `cb` (또는 `colcon build`) 명령어로 빌드를 진행한 후, 터미널을 껐다 켜거나 `sr` 명령어로 환경 변수를 갱신해야 적용됩니다.
*   현재 Ubuntu 시스템의 통신 접속용 IP 주소는 **`192.168.23.130` (포트: 10000)** 입니다. Windows Unity 세팅 시 참고하세요.
