# Cowork.md

현재 즉시 작업만 기록한다. 완료 이력은 `Current.md`로 이동한다.

## Active Task

Unity에서 Sphere(`Target_Handle`) 조작 시, 그리퍼 자세(Orientation)가 무조건 Y축 정렬 상태로 고정되는 현상 디버깅 및 좌표계 정합성 정밀 갱신.

## 방금 작업한 내용

- **물리 롤백 완료:** 무리한 1터미널 기동 통합 런치 기획을 철회하고 `vrobot_vr_full.launch.py` 파일을 영구 삭제 및 빌드 클리닝 완료.
- **2터미널 가동 복원 완료:** 기존에 검증되었던 2터미널 방식(`vrobot_full_sim.launch.py` + `unity_control.launch.py`)으로 완벽히 원복.
- **Clamping 완전 제거 완료:** 사용자의 자유로운 구동 테스트를 방해하지 않기 위해 C# 스크립트의 이동 한계 제한 코드 및 ROS2 파라미터를 완전 삭제/원복.

## 새 실행 방식

반드시 **2개의 개별 터미널**에서 아래 순서대로 런치를 가동합니다.

터미널 1: 로봇 본체 시뮬레이션
```bash
cd ~/Vrobot_ws
source install/setup.bash
ros2 launch vrobot_description vrobot_full_sim.launch.py
```

터미널 2: ROS-TCP 브리지 및 제어 핸들러 (10초 대기 후 실행 권장)
```bash
cd ~/Vrobot_ws
source install/setup.bash
ros2 launch vrobot_command unity_control.launch.py execute_enabled:=true
```

---

## 🚀 Coder Agent를 위한 즉각적인 실행 태스크

### 1. Unity ➔ ROS2 회전 쿼터니언 변환 분석
*   현재 [VrCommandPublisher.cs](file:///home/pyw/Vrobot_ws/unity_scripts/VrCommandPublisher.cs) L159-L164에 적용된 Unity ➔ ROS2 쿼터니언 변환식:
    ```csharp
    QuaternionMsg rosRotation = new QuaternionMsg(
        unityRotation.z,
        -unityRotation.x,
        unityRotation.y,
        -unityRotation.w
    );
    ```
*   현재 그리퍼 머리가 항상 Y축 정렬 자세로 고정되는 문제(혹은 특정 축 회전이 막히는 문제)가 이 쿼터니언 매핑 변환식 혹은 Unity 상의 조종 구체 축 제한과 연관되어 있는지 역산하여 디버깅을 시작합니다.
*   우선 Windows Unity 측 에디터의 `Target_Handle` 구체를 회전시켰을 때, 우분투의 `/vr/pose_goal` 토픽에 실시간 회전 변화가 쿼터니언 값에 제대로 반영되는지(모든 축이 변동되는지) 조사하십시오.
