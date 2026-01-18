#!/usr/bin/env python3
"""
Robot Control - Desktop Application
Uses PyQt6 for native desktop UI
"""

import sys
import cv2
import zmq
import time
import threading
import numpy as np
import roslibpy
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QLabel, QPushButton, QSlider, QFrame,
                              QGridLayout, QCheckBox, QGroupBox)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap, QFont, QKeyEvent
from ultralytics import YOLO

ROBOT = 'robot.local'


class RobotState(QObject):
    update_signal = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.connected = False
        self.mapping = False
        self.autonomous = False
        self.emergency = False
        self.speed = 0.15
        self.video_frame = None
        self.map_frame = None
        self.fps = 0
        self.action = "Ready"


class RobotController:
    def __init__(self, state):
        self.state = state
        self.ros = None
        self.manual_topic = None
        self.estop_topic = None
        self.explore_topic = None
        self.held_key = None
        
    def connect(self):
        def run():
            while True:
                try:
                    self.ros = roslibpy.Ros(host=ROBOT, port=9090)
                    self.ros.on_ready(self.on_ready)
                    self.ros.run()
                except Exception as e:
                    print(f"ROS connection error: {e}")
                    self.state.connected = False
                time.sleep(3)
        threading.Thread(target=run, daemon=True).start()
    
    def on_ready(self):
        print("✅ ROS Connected")
        self.state.connected = True
        
        self.manual_topic = roslibpy.Topic(self.ros, '/manual_cmd', 'geometry_msgs/Twist')
        self.manual_topic.advertise()
        
        self.estop_topic = roslibpy.Topic(self.ros, '/emergency_stop', 'std_msgs/Bool')
        self.estop_topic.advertise()
        
        self.explore_topic = roslibpy.Topic(self.ros, '/explore_enable', 'std_msgs/Bool')
        self.explore_topic.advertise()
        
        map_topic = roslibpy.Topic(self.ros, '/map', 'nav_msgs/OccupancyGrid')
        map_topic.subscribe(self.on_map)
    
    def on_map(self, msg):
        if not self.state.mapping:
            return
        try:
            w = msg['info']['width']
            h = msg['info']['height']
            data = np.array(msg['data'], dtype=np.int8).reshape((h, w))
            
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[data == -1] = [40, 40, 50]
            img[data == 0] = [230, 235, 240]
            img[data == 100] = [60, 60, 180]
            
            img = np.flipud(img)
            
            # Scale
            scale = max(2, 500 // max(w, h))
            img = cv2.resize(img, (w*scale, h*scale), interpolation=cv2.INTER_NEAREST)
            
            # Robot marker
            cx, cy = img.shape[1]//2, img.shape[0]//2
            cv2.rectangle(img, (cx-15, cy-20), (cx+15, cy+20), (0, 180, 0), -1)
            cv2.rectangle(img, (cx-15, cy-20), (cx+15, cy+20), (0, 255, 0), 2)
            pts = np.array([[cx, cy-30], [cx-10, cy-15], [cx+10, cy-15]], np.int32)
            cv2.fillPoly(img, [pts], (0, 255, 255))
            
            self.state.map_frame = img
            self.state.update_signal.emit()
        except Exception as e:
            print(f"Map error: {e}")
    
    def send_twist(self, linear, angular):
        if self.manual_topic and self.state.connected and not self.state.emergency:
            self.manual_topic.publish(roslibpy.Message({
                'linear': {'x': linear, 'y': 0, 'z': 0},
                'angular': {'x': 0, 'y': 0, 'z': angular}
            }))
    
    def stop(self):
        self.held_key = None
        self.send_twist(0, 0)
        self.state.action = "Stopped"
    
    def emergency_stop(self):
        self.state.emergency = True
        self.state.action = "🛑 EMERGENCY"
        if self.estop_topic:
            self.estop_topic.publish(roslibpy.Message({'data': True}))
        self.send_twist(0, 0)
    
    def release_emergency(self):
        self.state.emergency = False
        self.state.action = "Ready"
        if self.estop_topic:
            self.estop_topic.publish(roslibpy.Message({'data': False}))
    
    def set_autonomous(self, enabled):
        self.state.autonomous = enabled
        if self.explore_topic:
            self.explore_topic.publish(roslibpy.Message({'data': enabled}))
        self.state.action = "AUTO" if enabled else "Manual"
    
    def execute_key(self):
        if self.held_key == 'F':
            self.send_twist(self.state.speed, 0)
        elif self.held_key == 'B':
            self.send_twist(-self.state.speed, 0)
        elif self.held_key == 'L':
            self.send_twist(0, 0.5)
        elif self.held_key == 'R':
            self.send_twist(0, -0.5)


class VideoThread(threading.Thread):
    def __init__(self, state):
        super().__init__(daemon=True)
        self.state = state
        self.yolo = YOLO("../models/yolov8n.pt")
        
    def run(self):
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt_string(zmq.SUBSCRIBE, '')
        sock.setsockopt(zmq.RCVTIMEO, 5000)
        
        while True:
            try:
                sock.connect(f"tcp://{ROBOT}:5555")
                print("📷 Camera Connected")
                break
            except:
                time.sleep(1)
        
        fc = 0
        lt = time.time()
        
        while True:
            try:
                data = sock.recv()
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    results = self.yolo(frame, verbose=False, conf=0.5)
                    annotated = results[0].plot()
                    self.state.video_frame = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
                    fc += 1
                    
                    now = time.time()
                    if now - lt >= 1:
                        self.state.fps = fc
                        fc = 0
                        lt = now
                    
                    self.state.update_signal.emit()
            except:
                time.sleep(0.05)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.state = RobotState()
        self.controller = RobotController(self.state)
        
        self.setWindowTitle("Rescue Robot Control")
        self.setStyleSheet("""
            QMainWindow { background: #0d1117; }
            QLabel { color: white; }
            QGroupBox { 
                color: white; 
                border: 1px solid #30363d; 
                border-radius: 8px; 
                padding: 15px;
                margin-top: 10px;
                background: #161b22;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton { 
                background: #238636; 
                color: white; 
                border: none; 
                padding: 10px 20px; 
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background: #2ea043; }
            QPushButton:pressed { background: #1a7f37; }
            QPushButton#stop { background: #da3633; }
            QPushButton#stop:hover { background: #f85149; }
            QCheckBox { color: white; }
            QSlider::groove:horizontal { background: #30363d; height: 8px; border-radius: 4px; }
            QSlider::handle:horizontal { background: #58a6ff; width: 20px; margin: -6px 0; border-radius: 10px; }
        """)
        
        self.setup_ui()
        self.state.update_signal.connect(self.update_display)
        
        # Timers
        self.cmd_timer = QTimer()
        self.cmd_timer.timeout.connect(self.send_command)
        self.cmd_timer.start(80)
        
        self.controller.connect()
        VideoThread(self.state).start()
    
    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setSpacing(15)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # LEFT: Controls
        left = QVBoxLayout()
        left.setSpacing(10)
        
        # Status
        status_group = QGroupBox("STATUS")
        status_layout = QVBoxLayout(status_group)
        self.status_label = QLabel("Ready")
        self.status_label.setFont(QFont("Arial", 24, QFont.Weight.Bold))
        self.status_label.setStyleSheet("color: #58a6ff;")
        status_layout.addWidget(self.status_label)
        self.conn_label = QLabel("Connecting...")
        self.conn_label.setStyleSheet("color: #f85149;")
        status_layout.addWidget(self.conn_label)
        left.addWidget(status_group)
        
        # Controls
        ctrl_group = QGroupBox("CONTROLS")
        ctrl_layout = QGridLayout(ctrl_group)
        
        self.btn_f = QPushButton("▲")
        self.btn_f.setFixedSize(60, 60)
        self.btn_f.setFont(QFont("Arial", 24))
        self.btn_f.setStyleSheet("background: #1f6feb;")
        
        self.btn_l = QPushButton("◀")
        self.btn_l.setFixedSize(60, 60)
        self.btn_l.setFont(QFont("Arial", 24))
        self.btn_l.setStyleSheet("background: #1f6feb;")
        
        self.btn_s = QPushButton("■")
        self.btn_s.setFixedSize(60, 60)
        self.btn_s.setFont(QFont("Arial", 24))
        self.btn_s.setStyleSheet("background: #484f58;")
        
        self.btn_r = QPushButton("▶")
        self.btn_r.setFixedSize(60, 60)
        self.btn_r.setFont(QFont("Arial", 24))
        self.btn_r.setStyleSheet("background: #1f6feb;")
        
        self.btn_b = QPushButton("▼")
        self.btn_b.setFixedSize(60, 60)
        self.btn_b.setFont(QFont("Arial", 24))
        self.btn_b.setStyleSheet("background: #1f6feb;")
        
        ctrl_layout.addWidget(self.btn_f, 0, 1)
        ctrl_layout.addWidget(self.btn_l, 1, 0)
        ctrl_layout.addWidget(self.btn_s, 1, 1)
        ctrl_layout.addWidget(self.btn_r, 1, 2)
        ctrl_layout.addWidget(self.btn_b, 2, 1)
        
        self.btn_f.pressed.connect(lambda: self.key_press('F'))
        self.btn_f.released.connect(self.key_release)
        self.btn_l.pressed.connect(lambda: self.key_press('L'))
        self.btn_l.released.connect(self.key_release)
        self.btn_r.pressed.connect(lambda: self.key_press('R'))
        self.btn_r.released.connect(self.key_release)
        self.btn_b.pressed.connect(lambda: self.key_press('B'))
        self.btn_b.released.connect(self.key_release)
        self.btn_s.clicked.connect(self.controller.stop)
        
        left.addWidget(ctrl_group)
        
        # Speed
        speed_group = QGroupBox("SPEED")
        speed_layout = QVBoxLayout(speed_group)
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(10, 30)
        self.speed_slider.setValue(15)
        self.speed_slider.valueChanged.connect(lambda v: setattr(self.state, 'speed', v/100))
        speed_layout.addWidget(self.speed_slider)
        self.speed_label = QLabel("0.15 m/s")
        self.speed_slider.valueChanged.connect(lambda v: self.speed_label.setText(f"{v/100:.2f} m/s"))
        speed_layout.addWidget(self.speed_label)
        left.addWidget(speed_group)
        
        # Toggles
        toggle_group = QGroupBox("MODES")
        toggle_layout = QVBoxLayout(toggle_group)
        self.map_check = QCheckBox("Mapping")
        self.map_check.stateChanged.connect(lambda s: setattr(self.state, 'mapping', s == 2))
        toggle_layout.addWidget(self.map_check)
        self.auto_check = QCheckBox("Autonomous")
        self.auto_check.stateChanged.connect(lambda s: self.controller.set_autonomous(s == 2))
        toggle_layout.addWidget(self.auto_check)
        left.addWidget(toggle_group)
        
        # Emergency
        emergency_group = QGroupBox("EMERGENCY")
        emergency_layout = QHBoxLayout(emergency_group)
        self.stop_btn = QPushButton("🛑 STOP")
        self.stop_btn.setObjectName("stop")
        self.stop_btn.clicked.connect(self.controller.emergency_stop)
        emergency_layout.addWidget(self.stop_btn)
        self.release_btn = QPushButton("✓ Release")
        self.release_btn.clicked.connect(self.controller.release_emergency)
        emergency_layout.addWidget(self.release_btn)
        left.addWidget(emergency_group)
        
        left.addStretch()
        layout.addLayout(left)
        
        # CENTER: Video
        video_group = QGroupBox("LIVE CAMERA + YOLO")
        video_layout = QVBoxLayout(video_group)
        self.video_label = QLabel()
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background: #0d1117; border-radius: 8px;")
        video_layout.addWidget(self.video_label)
        self.fps_label = QLabel("-- FPS")
        self.fps_label.setStyleSheet("color: #3fb950;")
        video_layout.addWidget(self.fps_label)
        layout.addWidget(video_group, stretch=2)
        
        # RIGHT: Map
        map_group = QGroupBox("SLAM MAP")
        map_layout = QVBoxLayout(map_group)
        self.map_label = QLabel("Enable Mapping")
        self.map_label.setMinimumSize(500, 400)
        self.map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_label.setStyleSheet("background: #161b22; border-radius: 8px;")
        map_layout.addWidget(self.map_label)
        
        clear_btn = QPushButton("🗑️ Clear")
        clear_btn.clicked.connect(self.clear_map)
        map_layout.addWidget(clear_btn)
        
        layout.addWidget(map_group, stretch=2)
    
    def key_press(self, key):
        self.controller.held_key = key
        self.state.action = f"Moving {key}"
    
    def key_release(self):
        self.controller.stop()
    
    def send_command(self):
        if self.controller.held_key:
            self.controller.execute_key()
    
    def clear_map(self):
        self.state.map_frame = None
        self.map_label.setText("Map Cleared")
    
    def update_display(self):
        # Video
        if self.state.video_frame is not None:
            h, w, c = self.state.video_frame.shape
            qimg = QImage(self.state.video_frame.data, w, h, w*c, QImage.Format.Format_RGB888)
            scaled = qimg.scaled(self.video_label.size(), Qt.AspectRatioMode.KeepAspectRatio)
            self.video_label.setPixmap(QPixmap.fromImage(scaled))
        
        # Map
        if self.state.map_frame is not None:
            img = cv2.cvtColor(self.state.map_frame, cv2.COLOR_BGR2RGB)
            h, w, c = img.shape
            qimg = QImage(img.data, w, h, w*c, QImage.Format.Format_RGB888)
            scaled = qimg.scaled(self.map_label.size(), Qt.AspectRatioMode.KeepAspectRatio)
            self.map_label.setPixmap(QPixmap.fromImage(scaled))
        
        # Status
        self.status_label.setText(self.state.action)
        if self.state.connected:
            self.conn_label.setText("● Connected")
            self.conn_label.setStyleSheet("color: #3fb950;")
        else:
            self.conn_label.setText("● Disconnected")
            self.conn_label.setStyleSheet("color: #f85149;")
        
        self.fps_label.setText(f"{self.state.fps} FPS")
    
    def keyPressEvent(self, event: QKeyEvent):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in [Qt.Key.Key_W, Qt.Key.Key_Up]:
            self.key_press('F')
        elif key in [Qt.Key.Key_S, Qt.Key.Key_Down]:
            self.key_press('B')
        elif key in [Qt.Key.Key_A, Qt.Key.Key_Left]:
            self.key_press('L')
        elif key in [Qt.Key.Key_D, Qt.Key.Key_Right]:
            self.key_press('R')
        elif key == Qt.Key.Key_Space:
            self.controller.stop()
        elif key == Qt.Key.Key_Escape:
            self.controller.emergency_stop()
    
    def keyReleaseEvent(self, event: QKeyEvent):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in [Qt.Key.Key_W, Qt.Key.Key_Up, Qt.Key.Key_S, Qt.Key.Key_Down,
                   Qt.Key.Key_A, Qt.Key.Key_Left, Qt.Key.Key_D, Qt.Key.Key_Right]:
            self.controller.stop()


def main():
    print("Loading YOLO model...")
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
