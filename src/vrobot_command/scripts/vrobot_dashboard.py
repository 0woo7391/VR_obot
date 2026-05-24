#!/usr/bin/env python3
import sys
import os
import subprocess
import threading
import time
import re
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QTextEdit, QFrame, QGridLayout, QGroupBox
)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt, QProcess, QThread, QProcessEnvironment
from PyQt5.QtGui import QFont, QColor, QPalette

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# DDS 충돌 방지를 위한 격리형 토픽/노드 텍스트 감시 QThread (rclpy 미사용)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class TopicWatcherThread(QThread):
    status_received = pyqtSignal(str)
    nodes_updated = pyqtSignal(list)
    ping_updated = pyqtSignal(bool, float)

    def __init__(self, node_ip):
        super().__init__()
        self.node_ip = node_ip
        self.running = True
        self.echo_process = None

    def run(self):
        env = os.environ.copy()
        loop_count = 0

        self._spawn_echo_process(env)

        while self.running:
            if self.echo_process is None or self.echo_process.poll() is not None:
                self._spawn_echo_process(env)

            if self.echo_process and self.echo_process.poll() is None:
                try:
                    line = self.echo_process.stdout.readline()
                    if line:
                        val = line.strip().replace('"', '')
                        # WARNING 경고 텍스트는 필터링
                        if val and val != "data" and "WARNING" not in val:
                            self.status_received.emit(val)
                except Exception:
                    pass
            
            loop_count += 1
            
            # 2초 주기로 ros2 node list 텍스트 스캔
            if loop_count % 40 == 0:
                self._scan_active_nodes_cli(env)
                
            # 3초 주기로 Unity 포트 10000 소켓 검사
            if loop_count % 60 == 0:
                self._check_unity_socket_connection()
                loop_count = 0

            time.sleep(0.05)

        self._kill_echo_process()

    def _spawn_echo_process(self, env):
        try:
            self.echo_process = subprocess.Popen(
                ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/local_setup.bash && ros2 topic echo /vr/command_status --csv"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
                env=env,
                cwd="/home/pyw/Vrobot_ws"
            )
            os.set_blocking(self.echo_process.stdout.fileno(), False)
        except Exception:
            self.echo_process = None

    def _kill_echo_process(self):
        if self.echo_process:
            try:
                self.echo_process.terminate()
                self.echo_process.wait(timeout=1.0)
            except Exception:
                try:
                    self.echo_process.kill()
                except Exception:
                    pass
            self.echo_process = None

    def _scan_active_nodes_cli(self, env):
        t = threading.Thread(target=self._scan_nodes_job, args=(env,), daemon=True)
        t.start()

    def _scan_nodes_job(self, env):
        try:
            res = subprocess.run(
                ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/local_setup.bash && ros2 node list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                env=env,
                cwd="/home/pyw/Vrobot_ws",
                timeout=1.5
            )
            nodes = res.stdout.strip().split('\n')
            self.nodes_updated.emit(nodes)
        except Exception:
            pass

    def _check_unity_socket_connection(self):
        t = threading.Thread(target=self._socket_check_job, daemon=True)
        t.start()

    def _socket_check_job(self):
        try:
            res = subprocess.run(
                ["ss", "-nt", "state", "established", "sport", "=", ":10000"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            lines = res.stdout.strip().split('\n')
            connected = len(lines) > 1
            self.ping_updated.emit(connected, 0.0)
        except Exception:
            self.ping_updated.emit(False, 0.0)

    def stop(self):
        self.running = False
        self.wait()


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# PyQt5 세련된 다크 테마 대시보드 윈도우
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class VRobotDashboard(QMainWindow):
    def __init__(self, robot_ip):
        super().__init__()
        self.robot_ip = robot_ip
        
        self.setWindowTitle("VRobot Integration Dashboard")
        self.resize(1000, 800)
        
        # QProcess 객체 생성 (비동기 런치 실행용)
        self.process_launch1 = QProcess(self)
        self.process_launch2 = QProcess(self)

        self.process_launch1.readyReadStandardOutput.connect(lambda: self._read_stdout(self.process_launch1, "[Launch 1]"))
        self.process_launch1.readyReadStandardError.connect(lambda: self._read_stderr(self.process_launch1, "[Launch 1 ERROR]"))
        self.process_launch2.readyReadStandardOutput.connect(lambda: self._read_stdout(self.process_launch2, "[Launch 2]"))
        self.process_launch2.readyReadStandardError.connect(lambda: self._read_stderr(self.process_launch2, "[Launch 2 ERROR]"))
        
        self.auto_run_active = False
        self.system_active = False

        # 다크/네온 스타일시트 적용
        self.setStyleSheet("""
            QMainWindow {
                background-color: #121216;
            }
            QWidget {
                color: #E0E0E6;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QFrame#card {
                background-color: #1E1E24;
                border: 1px solid #2D2D35;
                border-radius: 12px;
            }
            QLabel#title {
                color: #00E676;
                font-size: 20px;
                font-weight: bold;
            }
            QLabel#subtitle {
                color: #9E9EAF;
                font-size: 12px;
            }
            QLabel#status_label {
                font-size: 15px;
                font-weight: bold;
            }
            QGroupBox {
                border: 1px solid #3D3D4A;
                border-radius: 8px;
                margin-top: 15px;
                font-weight: bold;
                color: #29B6F6;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }
            QTextEdit {
                background-color: #16161B;
                border: 1px solid #2D2D35;
                border-radius: 6px;
                color: #A9FFB2;
                font-family: 'Consolas', monospace;
                font-size: 12px;
            }
            QSlider::groove:horizontal {
                border: 1px solid #3E3E4F;
                height: 6px;
                background: #1A1A22;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #555566;
                width: 14px;
                margin: -4px 0;
                border-radius: 7px;
            }
            QPushButton {
                background-color: #2D2D3A;
                border: 1px solid #4D4D5A;
                border-radius: 6px;
                padding: 6px 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #3D3D4A;
                border: 1px solid #00E676;
            }
            QPushButton:pressed {
                background-color: #1E1E28;
            }
            QPushButton:disabled {
                background-color: #24242B;
                color: #666675;
                border: 1px solid #33333A;
            }
            
            /* 단계 표시등 인디케이터 스타일 */
            QLabel#step_idle, QLabel#step_plan, QLabel#step_execute, QLabel#step_fail {
                background-color: #1A1A22;
                border: 1px solid #3E3E4F;
                border-radius: 6px;
                padding: 10px 4px;
                font-weight: bold;
                font-size: 13px;
                color: #666675;
            }
        """)

        self._init_ui()
        self._start_watcher_thread()

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(15, 15, 15, 15)

        left_panel = QVBoxLayout()
        main_layout.addLayout(left_panel, stretch=6)

        # 1. 런처 카드
        launch_card = QFrame()
        launch_card.setObjectName("card")
        launch_layout = QVBoxLayout(launch_card)
        
        launch_title = QLabel("System Launcher Control")
        launch_title.setObjectName("title")
        launch_layout.addWidget(launch_title)

        btn_layout = QHBoxLayout()
        self.btn_auto_run = QPushButton("🚀 START INTEGRATION")
        self.btn_auto_run.setStyleSheet("background-color: #0E4225; border: 1.5px solid #00E676; color: #00E676; font-size: 13px;")
        
        self.btn_run_launch1 = QPushButton("1. Launch Robot")
        self.btn_run_launch2 = QPushButton("2. Launch Unity")
        
        self.btn_kill_all = QPushButton("💥 Force Kill All")
        self.btn_kill_all.setStyleSheet("background-color: #B71C1C; color: #FFF; border: 1px solid #FF5252;")
        
        self.btn_auto_run.clicked.connect(self._trigger_auto_run)
        self.btn_run_launch1.clicked.connect(self._toggle_launch1)
        self.btn_run_launch2.clicked.connect(self._toggle_launch2)
        self.btn_kill_all.clicked.connect(self._force_kill_all)
        
        btn_layout.addWidget(self.btn_auto_run)
        btn_layout.addWidget(self.btn_run_launch1)
        btn_layout.addWidget(self.btn_run_launch2)
        btn_layout.addWidget(self.btn_kill_all)
        launch_layout.addLayout(btn_layout)
        
        left_panel.addWidget(launch_card)

        # 2. 모션 단계
        step_card = QFrame()
        step_card.setObjectName("card")
        step_layout = QVBoxLayout(step_card)

        step_title = QLabel("Task Execution Phase")
        step_title.setObjectName("title")
        step_layout.addWidget(step_title)

        self.flow_layout = QHBoxLayout()
        self.flow_layout.setSpacing(10)

        self.lbl_step_idle = QLabel("IDLE")
        self.lbl_step_idle.setObjectName("step_idle")
        self.lbl_step_idle.setAlignment(Qt.AlignCenter)

        self.lbl_step_plan = QLabel("PLAN")
        self.lbl_step_plan.setObjectName("step_plan")
        self.lbl_step_plan.setAlignment(Qt.AlignCenter)

        self.lbl_step_execute = QLabel("EXECUTE")
        self.lbl_step_execute.setObjectName("step_execute")
        self.lbl_step_execute.setAlignment(Qt.AlignCenter)

        self.lbl_step_fail = QLabel("FAIL")
        self.lbl_step_fail.setObjectName("step_fail")
        self.lbl_step_fail.setAlignment(Qt.AlignCenter)

        self.flow_layout.addWidget(self.lbl_step_idle)
        self.flow_layout.addWidget(self.lbl_step_plan)
        self.flow_layout.addWidget(self.lbl_step_execute)
        self.flow_layout.addWidget(self.lbl_step_fail)
        
        step_layout.addLayout(self.flow_layout)
        left_panel.addWidget(step_card)

        # 3. IP 정보
        ip_card = QFrame()
        ip_card.setObjectName("card")
        ip_layout = QVBoxLayout(ip_card)
        
        lbl_title = QLabel("Device Connectivity")
        lbl_title.setObjectName("title")
        ip_layout.addWidget(lbl_title)
        
        self.lbl_ip = QLabel(f"Unity Peer IP Address : {self.robot_ip}")
        self.lbl_ip.setStyleSheet("font-size: 14px; font-weight: bold; margin-bottom: 2px;")
        ip_layout.addWidget(self.lbl_ip)
        
        left_panel.addWidget(ip_card)

        # 4. 노드 상태 감시
        nodes_card = QFrame()
        nodes_card.setObjectName("card")
        nodes_layout = QVBoxLayout(nodes_card)
        
        nodes_title = QLabel("System Nodes & Session Status")
        nodes_title.setObjectName("title")
        nodes_layout.addWidget(nodes_title)

        self.lbl_node_moveit = QLabel("🔴 move_group (Offline)")
        self.lbl_node_endpoint = QLabel("🔴 unity_endpoint (Offline)")
        self.lbl_node_handler = QLabel("🔴 vr_command_handler (Offline)")
        self.lbl_node_runtime = QLabel("🔴 unity_runtime (Offline)")
        
        for lbl in [self.lbl_node_moveit, self.lbl_node_endpoint, self.lbl_node_handler, self.lbl_node_runtime]:
            lbl.setStyleSheet("font-size: 13px; margin: 3px 0;")
            nodes_layout.addWidget(lbl)
            
        left_panel.addWidget(nodes_card)

        # 5. 콘솔 로그
        log_card = QFrame()
        log_card.setObjectName("card")
        log_layout = QVBoxLayout(log_card)
        
        log_title = QLabel("System Output Console Log")
        log_title.setObjectName("title")
        log_layout.addWidget(log_title)
        
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.append("[SYSTEM] Monitoring daemon started. Awaiting signals...")
        log_layout.addWidget(self.txt_log)
        
        left_panel.addWidget(log_card)

        # 우측 패널 (파라미터 튜닝)
        right_panel = QVBoxLayout()
        main_layout.addLayout(right_panel, stretch=4)

        control_card = QFrame()
        control_card.setObjectName("card")
        control_layout = QVBoxLayout(control_card)

        ctrl_title = QLabel("Parameter Tuning")
        ctrl_title.setObjectName("title")
        control_layout.addWidget(ctrl_title)

        lbl_desc = QLabel("※ These parameters are currently LOCKED.\nThey will activate once kinematic alignment is verified.")
        lbl_desc.setObjectName("subtitle")
        lbl_desc.setWordWrap(True)
        control_layout.addWidget(lbl_desc)

        grp_tuning = QGroupBox("Dynamic Scale (LOCKED)")
        grp_layout = QGridLayout(grp_tuning)
        
        grp_layout.addWidget(QLabel("Joint Speed Scale"), 0, 0)
        self.slide_j_speed = QSlider(Qt.Horizontal)
        self.slide_j_speed.setEnabled(False)
        grp_layout.addWidget(self.slide_j_speed, 0, 1)

        grp_layout.addWidget(QLabel("Joint Accel Scale"), 1, 0)
        self.slide_j_acc = QSlider(Qt.Horizontal)
        self.slide_j_acc.setEnabled(False)
        grp_layout.addWidget(self.slide_j_acc, 1, 1)

        grp_layout.addWidget(QLabel("Cartesian Speed Scale"), 2, 0)
        self.slide_c_speed = QSlider(Qt.Horizontal)
        self.slide_c_speed.setEnabled(False)
        grp_layout.addWidget(self.slide_c_speed, 2, 1)

        control_layout.addWidget(grp_tuning)

        grp_hardware = QGroupBox("Hardware Control (LOCKED)")
        hw_layout = QGridLayout(grp_hardware)

        self.btn_servo_on = QPushButton("Servo ON")
        self.btn_servo_off = QPushButton("Servo OFF")
        self.btn_alarm_reset = QPushButton("Alarm Reset")
        self.btn_backdrive = QPushButton("Backdrive Mode")

        for btn in [self.btn_servo_on, self.btn_servo_off, self.btn_alarm_reset, self.btn_backdrive]:
            btn.setEnabled(False)

        hw_layout.addWidget(self.btn_servo_on, 0, 0)
        hw_layout.addWidget(self.btn_servo_off, 0, 1)
        hw_layout.addWidget(self.btn_alarm_reset, 1, 0)
        hw_layout.addWidget(self.btn_backdrive, 1, 1)

        control_layout.addWidget(grp_hardware)
        
        grp_joints = QGroupBox("Joint Positions (LOCKED)")
        joint_layout = QGridLayout(grp_joints)
        for i in range(1, 7):
            joint_layout.addWidget(QLabel(f"J{i} :"), (i-1)//2, ((i-1)%2)*2)
            lbl_val = QLabel("0.00°")
            lbl_val.setStyleSheet("color: #666675;")
            joint_layout.addWidget(lbl_val, (i-1)//2, ((i-1)%2)*2 + 1)
        
        control_layout.addWidget(grp_joints)
        right_panel.addWidget(control_card)

        self._update_progress_ui("IDLE")

    def _start_watcher_thread(self):
        self.watcher_thread = TopicWatcherThread(self.robot_ip)
        self.watcher_thread.status_received.connect(self._on_status_received)
        self.watcher_thread.nodes_updated.connect(self._on_nodes_updated)
        self.watcher_thread.ping_updated.connect(self._on_ping_updated)
        self.watcher_thread.start()

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 자동 시퀀서 및 QProcess 제어 (부모 환경변수 완벽 상속 보장)
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    def _trigger_auto_run(self):
        if not self.auto_run_active and not self.system_active:
            self.txt_log.append("[AUTO-RUN] Initializing Auto-Run sequence. Starting Phase 1 (Robot & MoveIt)...")
            self.auto_run_active = True
            self.btn_auto_run.setText("⏳ RUNNING PHASE 1...")
            self.btn_auto_run.setStyleSheet("background-color: #A5D6A7; border: 1.5px solid #2E7D32; color: #1B5E20; font-size: 13px;")
            
            if self.process_launch1.state() == QProcess.NotRunning:
                self._start_launch1_process()
        else:
            self.txt_log.append("[AUTO-RUN] Stopping all active execution paths...")
            self._force_kill_all()
            self._reset_auto_run_ui()

    def _toggle_launch1(self):
        if self.process_launch1.state() == QProcess.NotRunning:
            self._start_launch1_process()
        else:
            self._stop_launch1_process()

    def _toggle_launch2(self):
        if self.process_launch2.state() == QProcess.NotRunning:
            self._start_launch2_process()
        else:
            self._stop_launch2_process()

    def _start_launch1_process(self):
        self.txt_log.append("[LAUNCHER] Launching Phase 1: Robot & MoveIt simulation...")
        self.btn_run_launch1.setText("⏹ Stop Robot")
        self.btn_run_launch1.setStyleSheet("background-color: #C62828; color: #FFF;")
        
        env = QProcessEnvironment.systemEnvironment()
        self.process_launch1.setProcessEnvironment(env)
        self.process_launch1.setWorkingDirectory("/home/pyw/Vrobot_ws")
        self.process_launch1.start("bash", ["-c", "source /opt/ros/humble/setup.bash && source install/local_setup.bash && ros2 launch vrobot_description vrobot_full_sim.launch.py"])

    def _stop_launch1_process(self):
        self.txt_log.append("[LAUNCHER] Stopping Phase 1 process...")
        self.process_launch1.terminate()
        self.process_launch1.waitForFinished(3000)
        self.btn_run_launch1.setText("1. Launch Robot")
        self.btn_run_launch1.setStyleSheet("")

    def _start_launch2_process(self):
        self.txt_log.append("[LAUNCHER] Launching Phase 2: Unity Mediator node...")
        self.btn_run_launch2.setText("⏹ Stop Unity")
        self.btn_run_launch2.setStyleSheet("background-color: #C62828; color: #FFF;")
        
        env = QProcessEnvironment.systemEnvironment()
        self.process_launch2.setProcessEnvironment(env)
        self.process_launch2.setWorkingDirectory("/home/pyw/Vrobot_ws")
        self.process_launch2.start("bash", ["-c", "source /opt/ros/humble/setup.bash && source install/local_setup.bash && ros2 launch vrobot_command unity_control.launch.py execute_enabled:=true"])

    def _stop_launch2_process(self):
        self.txt_log.append("[LAUNCHER] Stopping Phase 2 process...")
        self.process_launch2.terminate()
        self.process_launch2.waitForFinished(3000)
        self.btn_run_launch2.setText("2. Launch Unity")
        self.btn_run_launch2.setStyleSheet("")

    def _reset_auto_run_ui(self):
        self.auto_run_active = False
        self.system_active = False
        self.btn_auto_run.setText("🚀 START INTEGRATION")
        self.btn_auto_run.setStyleSheet("background-color: #0E4225; border: 1.5px solid #00E676; color: #00E676; font-size: 13px;")

    def _force_kill_all(self):
        self.txt_log.append("[ALERT] Executing Force Kill on all background ROS2 & Python processes...")
        self.btn_kill_all.setText("💥 Killing...")
        self.btn_kill_all.setEnabled(False)
        QApplication.processEvents()

        if self.process_launch1.state() != QProcess.NotRunning:
            self.process_launch1.kill()
            self.process_launch1.waitForFinished(1000)
        if self.process_launch2.state() != QProcess.NotRunning:
            self.process_launch2.kill()
            self.process_launch2.waitForFinished(1000)

        # 🚨 개선: 대시보드(자신)를 강제 사살하지 않도록 'python3' 전체 킬을 제거하고,
        # 10000번 포트 점유 좀비 세션(fuser -k) 및 관련 ROS2/Mediator 프로세스명을 지목하여 정밀 타격 킬
        cmd = "fuser -k 10000/tcp; killall -9 ros2 rviz2 robot_state_publisher ros2_control_node move_group; pkill -f ros_tcp_endpoint; pkill -f vr_command_handler; rm -f /dev/shm/fastrtps_port*"
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        time.sleep(1.5)
        
        self.btn_run_launch1.setText("1. Launch Robot")
        self.btn_run_launch1.setStyleSheet("")
        self.btn_run_launch2.setText("2. Launch Unity")
        self.btn_run_launch2.setStyleSheet("")
        
        self._reset_auto_run_ui()
        self.btn_kill_all.setText("💥 Force Kill All")
        self.btn_kill_all.setEnabled(True)
        self.txt_log.append("[SYSTEM] Cleanup complete. Memory cleared.")

    def _read_stdout(self, process, prefix):
        data = process.readAllStandardOutput().data().decode('utf-8', errors='ignore').strip()
        if data:
            for line in data.split('\n'):
                if line.strip():
                    self.txt_log.append(f"{prefix} {line.strip()[:120]}")

    def _read_stderr(self, process, prefix):
        data = process.readAllStandardError().data().decode('utf-8', errors='ignore').strip()
        if data:
            for line in data.split('\n'):
                if line.strip():
                    self.txt_log.append(f"<font color='#FF5252'>{prefix} {line.strip()[:120]}</font>")

    def _on_status_received(self, status):
        curr_time = time.strftime("%H:%M:%S")
        self.txt_log.append(f"[{curr_time}] [GOAL_STATUS] -> {status}")
        self._update_progress_ui(status)

    def _update_progress_ui(self, status):
        default_style = "background-color: #1A1A22; border: 1px solid #3E3E4F; color: #666675;"
        self.lbl_step_idle.setStyleSheet(default_style)
        self.lbl_step_plan.setStyleSheet(default_style)
        self.lbl_step_execute.setStyleSheet(default_style)
        self.lbl_step_fail.setStyleSheet(default_style)

        status_upper = status.upper()
        
        if "IDLE" in status_upper:
            self.lbl_step_idle.setStyleSheet("background-color: #2D2D3A; border: 1px solid #29B6F6; color: #29B6F6; font-weight: bold;")
        elif "ACCEPTED" in status_upper or "PLANNING" in status_upper or "PLANNED" in status_upper:
            self.lbl_step_plan.setStyleSheet("background-color: #01579B; border: 1px solid #29B6F6; color: #29B6F6; font-weight: bold;")
        elif "EXECUTING" in status_upper or "RUNNING" in status_upper or "MOVING" in status_upper or "EXECUTED" in status_upper or "SUCCESS" in status_upper or "FINISHED" in status_upper:
            self.lbl_step_execute.setStyleSheet("background-color: #1B5E20; border: 1px solid #00E676; color: #00E676; font-weight: bold;")
        elif "FAILED" in status_upper or "ERROR" in status_upper or ("REJECTED" in status_upper and "IN_PROGRESS" not in status_upper):
            self.lbl_step_fail.setStyleSheet("background-color: #B71C1C; border: 1px solid #FF5252; color: #FF5252; font-weight: bold;")

    def _on_nodes_updated(self, active_nodes):
        moveit_active = any('move_group' in n for n in active_nodes)

        if moveit_active:
            self.lbl_node_moveit.setText("🟢 move_group (MoveIt2 Core)")
            self.lbl_node_moveit.setStyleSheet("color: #00E676; font-size: 13px; font-weight: bold;")
            
            if self.auto_run_active and self.process_launch2.state() == QProcess.NotRunning:
                self.auto_run_active = False
                self.txt_log.append("[AUTO-RUN] Detected MoveIt2 core online (move_group). Spawning Phase 2 (Unity Mediator)...")
                self.btn_auto_run.setText("⏳ RUNNING PHASE 2...")
                self._start_launch2_process()
        else:
            self.lbl_node_moveit.setText("🔴 move_group (Offline)")
            self.lbl_node_moveit.setStyleSheet("color: #FF5252; font-size: 13px;")

        endpoint_active = any('ros_tcp_endpoint' in n for n in active_nodes)
        if endpoint_active:
            self.lbl_node_endpoint.setText("🟢 unity_endpoint (TCP Bridge)")
            self.lbl_node_endpoint.setStyleSheet("color: #00E676; font-size: 13px; font-weight: bold;")
            
            handler_active = any('vr_command_handler' in n for n in active_nodes)
            if not self.system_active and handler_active:
                self.txt_log.append("[AUTO-RUN] All system nodes are now ONLINE. Sequencer complete.")
                self.system_active = True
                self.btn_auto_run.setText("⏹ STOP INTEGRATION")
                self.btn_auto_run.setStyleSheet("background-color: #C62828; color: #FFF; font-size: 13px;")
        else:
            self.lbl_node_endpoint.setText("🔴 unity_endpoint (Offline)")
            self.lbl_node_endpoint.setStyleSheet("color: #FF5252; font-size: 13px;")

        if any('vr_command_handler' in n for n in active_nodes):
            self.lbl_node_handler.setText("🟢 vr_command_handler (Mediator)")
            self.lbl_node_handler.setStyleSheet("color: #00E676; font-size: 13px; font-weight: bold;")
        else:
            self.lbl_node_handler.setText("🔴 vr_command_handler (Offline)")
            self.lbl_node_handler.setStyleSheet("color: #FF5252; font-size: 13px;")

    def _on_ping_updated(self, success, duration):
        if success:
            self.lbl_node_runtime.setText("🟢 unity_runtime (Play Session)")
            self.lbl_node_runtime.setStyleSheet("color: #00E676; font-size: 13px; font-weight: bold;")
        else:
            self.lbl_node_runtime.setText("🔴 unity_runtime (Offline)")
            self.lbl_node_runtime.setStyleSheet("color: #FF5252; font-size: 13px;")

    def closeEvent(self, event):
        if hasattr(self, 'watcher_thread'):
            self.watcher_thread.stop()
            
        if self.process_launch1.state() != QProcess.NotRunning:
            self.process_launch1.terminate()
            self.process_launch1.waitForFinished(1000)
        if self.process_launch2.state() != QProcess.NotRunning:
            self.process_launch2.terminate()
            self.process_launch2.waitForFinished(1000)
            
        event.accept()


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 엔트리 포인트
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def main():
    ROBOT_IP = "110.120.1.50"
    
    app = QApplication(sys.argv)
    dashboard = VRobotDashboard(ROBOT_IP)
    dashboard.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
