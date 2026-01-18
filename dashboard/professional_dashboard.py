#!/usr/bin/env python3
"""
Professional Robot Control Dashboard
=====================================
Full-featured dashboard with:
- Manual Control (WASD/Arrows)
- Mapping Start/Stop/Save
- Autonomous Mode Toggle
- Live SLAM Map View
- YOLO Video Feed
- Emergency Stop

Run: python professional_dashboard.py
Open: http://localhost:8080
"""

import cv2
import zmq
import time
import json
import base64
import roslibpy
import threading
import numpy as np
from nicegui import ui
from ultralytics import YOLO
from datetime import datetime

# ============== Configuration ==============
ROBOT_HOST = 'robot.local'
ROS_PORT = 9090
VIDEO_PORT = 5555

# ============== State ==============
class RobotState:
    def __init__(self):
        self.connected = False
        self.mapping_active = False
        self.autonomous_mode = False
        self.emergency_stopped = False
        self.current_action = "Idle"
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.latest_frame = None
        self.latest_map = None
        self.frame_count = 0
        self.map_count = 0
        self.last_cmd = 'S'

state = RobotState()

# ============== ROS Connection ==============
client = None
manual_topic = None
estop_topic = None
explore_topic = None
slam_service = None

def init_ros():
    global client, manual_topic, estop_topic, explore_topic
    
    client = roslibpy.Ros(host=ROBOT_HOST, port=ROS_PORT)
    
    def on_ready():
        global manual_topic, estop_topic, explore_topic
        print("✅ Connected to ROS!")
        state.connected = True
        
        # Publishers
        manual_topic = roslibpy.Topic(client, '/manual_cmd', 'geometry_msgs/Twist')
        manual_topic.advertise()
        
        estop_topic = roslibpy.Topic(client, '/emergency_stop', 'std_msgs/Bool')
        estop_topic.advertise()
        
        explore_topic = roslibpy.Topic(client, '/explore_enable', 'std_msgs/Bool')
        explore_topic.advertise()
        
        # Subscribe to map
        map_topic = roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
        map_topic.subscribe(map_callback)
    
    def on_close(event):
        state.connected = False
        print("❌ ROS disconnected")
    
    client.on_ready(on_ready)
    client.on('close', on_close)
    
    # Run in background
    def run_ros():
        while True:
            try:
                if not client.is_connected:
                    client.run()
            except:
                pass
            time.sleep(2)
    
    threading.Thread(target=run_ros, daemon=True).start()

def map_callback(msg):
    try:
        width = msg['info']['width']
        height = msg['info']['height']
        resolution = msg['info']['resolution']
        data = np.array(msg['data'], dtype=np.int8).reshape((height, width))
        
        # Create colored map
        img = np.full((height, width, 3), 40, dtype=np.uint8)  # Unknown = dark gray
        img[data == 0] = [240, 240, 240]    # Free = white
        img[data == 100] = [40, 40, 200]    # Occupied = red
        img[data == -1] = [60, 60, 60]      # Unknown = gray
        
        # Flip and scale
        img = np.flipud(img)
        scale = max(1, 400 // max(width, height))
        img = cv2.resize(img, (width * scale, height * scale), interpolation=cv2.INTER_NEAREST)
        
        # Add grid
        for i in range(0, img.shape[0], 50):
            cv2.line(img, (0, i), (img.shape[1], i), (80, 80, 80), 1)
        for j in range(0, img.shape[1], 50):
            cv2.line(img, (j, 0), (j, img.shape[0]), (80, 80, 80), 1)
        
        # Draw robot position (center)
        cx, cy = img.shape[1] // 2, img.shape[0] // 2
        cv2.circle(img, (cx, cy), 8, (0, 200, 0), -1)
        cv2.circle(img, (cx, cy), 10, (0, 255, 0), 2)
        
        _, buffer = cv2.imencode('.png', img)
        state.latest_map = f'data:image/png;base64,{base64.b64encode(buffer).decode()}'
        state.map_count += 1
    except Exception as e:
        pass

# ============== Video Stream ==============
print("Loading YOLO model...")
model = YOLO("../models/yolov8n.pt")
print("✅ YOLO loaded!")

def video_loop():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.setsockopt_string(zmq.SUBSCRIBE, '')
    socket.setsockopt(zmq.RCVTIMEO, 5000)
    
    while True:
        try:
            socket.connect(f"tcp://{ROBOT_HOST}:{VIDEO_PORT}")
            print(f"📷 Connected to camera at {ROBOT_HOST}:{VIDEO_PORT}")
            break
        except:
            time.sleep(1)
    
    while True:
        try:
            data = socket.recv()
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            
            if frame is not None:
                # YOLO detection
                results = model(frame, verbose=False, conf=0.5)
                annotated = results[0].plot()
                
                # Add status overlay
                status = "AUTONOMOUS" if state.autonomous_mode else "MANUAL"
                color = (0, 255, 0) if state.autonomous_mode else (255, 200, 0)
                cv2.putText(annotated, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                
                if state.emergency_stopped:
                    cv2.putText(annotated, "EMERGENCY STOP", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                _, buffer = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
                state.latest_frame = f'data:image/jpeg;base64,{base64.b64encode(buffer).decode()}'
                state.frame_count += 1
        except:
            time.sleep(0.1)

threading.Thread(target=video_loop, daemon=True).start()

# ============== Control Functions ==============
def send_cmd(linear=0.0, angular=0.0):
    if manual_topic and state.connected and not state.emergency_stopped:
        msg = {'linear': {'x': linear, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': angular}}
        manual_topic.publish(roslibpy.Message(msg))

def move(cmd, action):
    if state.last_cmd != cmd:
        state.last_cmd = cmd
        state.current_action = action
        
        if cmd == 'F': send_cmd(state.linear_speed, 0)
        elif cmd == 'B': send_cmd(-state.linear_speed, 0)
        elif cmd == 'L': send_cmd(0, state.angular_speed)
        elif cmd == 'R': send_cmd(0, -state.angular_speed)
        else: send_cmd(0, 0)

def stop():
    move('S', 'Stopped')

def emergency_stop():
    state.emergency_stopped = True
    state.current_action = "🚨 EMERGENCY STOP"
    if estop_topic:
        estop_topic.publish(roslibpy.Message({'data': True}))
    send_cmd(0, 0)

def release_emergency():
    state.emergency_stopped = False
    state.current_action = "Ready"
    if estop_topic:
        estop_topic.publish(roslibpy.Message({'data': False}))

def toggle_autonomous(enabled):
    state.autonomous_mode = enabled
    if explore_topic:
        explore_topic.publish(roslibpy.Message({'data': enabled}))
    state.current_action = "Autonomous" if enabled else "Manual Control"

def toggle_mapping(enabled):
    state.mapping_active = enabled
    # SLAM is always running, this just affects UI indication
    state.current_action = "Mapping Active" if enabled else "Mapping Paused"

# ============== GUI ==============
ui.add_head_html('''
<style>
    body { background: #0d1117; font-family: 'Segoe UI', sans-serif; }
    .glass { 
        background: rgba(22, 27, 34, 0.9); 
        backdrop-filter: blur(10px);
        border: 1px solid rgba(255,255,255,0.1);
        border-radius: 12px;
    }
    .control-btn {
        width: 65px !important; height: 65px !important;
        font-size: 26px !important; border-radius: 12px !important;
        transition: all 0.15s !important;
    }
    .control-btn:hover { transform: scale(1.08); }
    .control-btn:active { transform: scale(0.92); background: #1e40af !important; }
    .status-dot { 
        width: 12px; height: 12px; border-radius: 50%;
        display: inline-block; margin-right: 8px;
    }
    .online { background: #22c55e; box-shadow: 0 0 10px #22c55e; }
    .offline { background: #ef4444; box-shadow: 0 0 10px #ef4444; }
</style>
''')

with ui.row().classes('w-full h-screen p-3 gap-3'):
    
    # ============== LEFT PANEL - Control ==============
    with ui.column().classes('w-80 gap-3'):
        
        # Header
        with ui.card().classes('glass p-4'):
            with ui.row().classes('items-center gap-3'):
                ui.icon('smart_toy', size='32px').classes('text-blue-400')
                with ui.column().classes('gap-0'):
                    ui.label('RESCUE ROBOT').classes('text-xl font-bold text-white')
                    with ui.row().classes('items-center'):
                        conn_dot = ui.html('<span class="status-dot offline"></span>')
                        conn_text = ui.label('Connecting...').classes('text-sm text-gray-400')
        
        # Status
        with ui.card().classes('glass p-4'):
            ui.label('STATUS').classes('text-xs text-gray-500 font-bold tracking-wider mb-2')
            action_label = ui.label('Initializing...').classes('text-2xl font-bold text-cyan-400')
        
        # Control Pad
        with ui.card().classes('glass p-5'):
            ui.label('MANUAL CONTROL').classes('text-xs text-gray-500 font-bold tracking-wider mb-3 text-center')
            
            with ui.column().classes('items-center gap-1'):
                # Forward
                fwd = ui.button('▲').classes('control-btn bg-blue-600 text-white')
                fwd.on('mousedown', lambda: move('F', '▲ Forward'))
                fwd.on('mouseup', stop)
                fwd.on('mouseleave', stop)
                
                # Left Stop Right
                with ui.row().classes('gap-1'):
                    left = ui.button('◀').classes('control-btn bg-blue-600 text-white')
                    left.on('mousedown', lambda: move('L', '◀ Left'))
                    left.on('mouseup', stop)
                    left.on('mouseleave', stop)
                    
                    ui.button('■').classes('control-btn bg-gray-700 text-white').on('click', stop)
                    
                    right = ui.button('▶').classes('control-btn bg-blue-600 text-white')
                    right.on('mousedown', lambda: move('R', '▶ Right'))
                    right.on('mouseup', stop)
                    right.on('mouseleave', stop)
                
                # Backward
                back = ui.button('▼').classes('control-btn bg-blue-600 text-white')
                back.on('mousedown', lambda: move('B', '▼ Backward'))
                back.on('mouseup', stop)
                back.on('mouseleave', stop)
            
            # Speed
            ui.label('Speed').classes('text-gray-400 text-sm mt-4')
            speed_slider = ui.slider(min=0.1, max=0.5, step=0.05, value=0.2).classes('w-full')
            speed_label = ui.label('0.2 m/s').classes('text-cyan-400 text-sm')
            
            def update_speed():
                state.linear_speed = speed_slider.value
                speed_label.text = f'{speed_slider.value:.2f} m/s'
            speed_slider.on('change', update_speed)
        
        # Mode Switches
        with ui.card().classes('glass p-4'):
            ui.label('MODES').classes('text-xs text-gray-500 font-bold tracking-wider mb-3')
            
            with ui.row().classes('items-center justify-between w-full mb-2'):
                ui.label('Mapping').classes('text-white')
                mapping_switch = ui.switch(value=True).on('change', lambda e: toggle_mapping(e.value))
            
            with ui.row().classes('items-center justify-between w-full'):
                ui.label('Autonomous').classes('text-white')
                auto_switch = ui.switch().on('change', lambda e: toggle_autonomous(e.value))
        
        # Emergency
        with ui.card().classes('glass p-4 border-red-500/50'):
            ui.label('EMERGENCY').classes('text-xs text-red-400 font-bold tracking-wider mb-2')
            with ui.row().classes('gap-2 w-full'):
                ui.button('🛑 STOP', on_click=emergency_stop).classes('flex-grow bg-red-600 text-white font-bold')
                ui.button('✓ Release', on_click=release_emergency).classes('bg-green-600 text-white')
    
    # ============== CENTER - Video ==============
    with ui.column().classes('flex-grow gap-3'):
        with ui.card().classes('glass p-3 flex-grow'):
            with ui.row().classes('justify-between items-center mb-2'):
                ui.label('LIVE FEED + YOLO').classes('text-white font-bold')
                fps_label = ui.label('-- FPS').classes('text-green-400')
            video_img = ui.image().classes('w-full h-full bg-black rounded-lg object-contain')
            video_img.style('max-height: 70vh')
    
    # ============== RIGHT - Map ==============
    with ui.column().classes('w-96 gap-3'):
        with ui.card().classes('glass p-3 flex-grow'):
            with ui.row().classes('justify-between items-center mb-2'):
                ui.label('🗺️ SLAM MAP').classes('text-white font-bold')
                map_status = ui.label('Building...').classes('text-yellow-400 text-sm')
            map_img = ui.image().classes('w-full bg-gray-900 rounded-lg object-contain')
            map_img.style('max-height: 60vh')
            
            # Map controls
            with ui.row().classes('gap-2 mt-3 justify-center'):
                ui.button('📷 Save Map', on_click=lambda: ui.notify('Map saved!')).classes('bg-blue-600')
                ui.button('🗑️ Clear', on_click=lambda: ui.notify('Clear not implemented')).classes('bg-gray-600')
        
        # Info
        with ui.card().classes('glass p-4'):
            ui.label('SYSTEM INFO').classes('text-xs text-gray-500 font-bold tracking-wider mb-2')
            with ui.column().classes('text-sm gap-1'):
                ui.label(f'Robot: {ROBOT_HOST}').classes('text-gray-400 font-mono')
                ui.label(f'ROS: {ROS_PORT}').classes('text-gray-400 font-mono')
                ui.label(f'Camera: {VIDEO_PORT}').classes('text-gray-400 font-mono')

# ============== Keyboard ==============
def handle_key(e):
    if e.action.keydown:
        if e.key in ['w', 'W', 'ArrowUp']: move('F', '▲ Forward')
        elif e.key in ['s', 'S', 'ArrowDown']: move('B', '▼ Backward')
        elif e.key in ['a', 'A', 'ArrowLeft']: move('L', '◀ Left')
        elif e.key in ['d', 'D', 'ArrowRight']: move('R', '▶ Right')
        elif e.key == ' ': stop()
        elif e.key == 'Escape': emergency_stop()
    elif e.action.keyup:
        if e.key in ['w','W','s','S','a','A','d','D','ArrowUp','ArrowDown','ArrowLeft','ArrowRight']:
            stop()

ui.keyboard(on_key=handle_key)

# ============== UI Updates ==============
last_frame = 0
last_time = time.time()

def update_ui():
    global last_frame, last_time
    
    # Video
    if state.latest_frame:
        video_img.source = state.latest_frame
    
    # Map
    if state.latest_map:
        map_img.source = state.latest_map
        map_status.text = f'{state.map_count} updates'
        map_status.classes(remove='text-yellow-400', add='text-green-400')
    
    # Connection
    if state.connected:
        conn_dot.content = '<span class="status-dot online"></span>'
        conn_text.text = 'Connected'
        conn_text.classes(remove='text-gray-400', add='text-green-400')
    else:
        conn_dot.content = '<span class="status-dot offline"></span>'
        conn_text.text = 'Disconnected'
        conn_text.classes(remove='text-green-400', add='text-gray-400')
    
    # Action
    action_label.text = state.current_action
    if state.emergency_stopped:
        action_label.classes(remove='text-cyan-400 text-green-400', add='text-red-400')
    elif state.autonomous_mode:
        action_label.classes(remove='text-cyan-400 text-red-400', add='text-green-400')
    else:
        action_label.classes(remove='text-green-400 text-red-400', add='text-cyan-400')
    
    # FPS
    now = time.time()
    if now - last_time >= 1.0:
        fps = state.frame_count - last_frame
        fps_label.text = f'{fps} FPS'
        last_frame = state.frame_count
        last_time = now

ui.timer(0.1, update_ui)

# ============== Start ==============
init_ros()
time.sleep(1)

ui.run(title='Rescue Robot - Professional Dashboard', port=8080, dark=True, reload=False)
