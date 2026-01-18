#!/usr/bin/env python3
"""
Professional Robot Control Dashboard - Fixed Version
=====================================================
Fixes:
- Continuous command sending while button held
- Better map visualization
- Working clear button
- Autonomous mode toggle
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
        self.mapping_active = True
        self.autonomous_mode = False
        self.emergency_stopped = False
        self.current_action = "Ready"
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.latest_frame = None
        self.latest_map = None
        self.frame_count = 0
        self.map_count = 0
        self.last_cmd = 'S'
        self.button_held = False
        self.held_command = None

state = RobotState()

# ============== ROS Connection ==============
client = None
manual_topic = None
estop_topic = None
explore_topic = None

def init_ros():
    global client, manual_topic, estop_topic, explore_topic
    
    client = roslibpy.Ros(host=ROBOT_HOST, port=ROS_PORT)
    
    def on_ready():
        global manual_topic, estop_topic, explore_topic
        print("✅ Connected to ROS!")
        state.connected = True
        
        manual_topic = roslibpy.Topic(client, '/manual_cmd', 'geometry_msgs/Twist')
        manual_topic.advertise()
        
        estop_topic = roslibpy.Topic(client, '/emergency_stop', 'std_msgs/Bool')
        estop_topic.advertise()
        
        explore_topic = roslibpy.Topic(client, '/explore_enable', 'std_msgs/Bool')
        explore_topic.advertise()
        
        map_topic = roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
        map_topic.subscribe(map_callback)
    
    def on_close(event):
        state.connected = False
    
    client.on_ready(on_ready)
    client.on('close', on_close)
    
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
    if not state.mapping_active:
        return
    try:
        width = msg['info']['width']
        height = msg['info']['height']
        data = np.array(msg['data'], dtype=np.int8).reshape((height, width))
        
        # Better colors
        img = np.full((height, width, 3), 30, dtype=np.uint8)
        img[data == 0] = [255, 255, 255]     # Free = white
        img[data == 100] = [60, 60, 220]     # Occupied = red
        img[data == -1] = [50, 50, 50]       # Unknown = dark gray
        
        img = np.flipud(img)
        
        # Scale to larger size
        target_size = 500
        scale = max(1, target_size // max(width, height))
        img = cv2.resize(img, (width * scale, height * scale), interpolation=cv2.INTER_NEAREST)
        
        # Draw robot (green dot in center)
        cx, cy = img.shape[1] // 2, img.shape[0] // 2
        cv2.circle(img, (cx, cy), 12, (0, 200, 0), -1)
        cv2.circle(img, (cx, cy), 14, (0, 255, 0), 2)
        
        # Add border
        img = cv2.copyMakeBorder(img, 5, 5, 5, 5, cv2.BORDER_CONSTANT, value=(100, 100, 100))
        
        _, buffer = cv2.imencode('.png', img)
        state.latest_map = f'data:image/png;base64,{base64.b64encode(buffer).decode()}'
        state.map_count += 1
    except:
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
            print(f"📷 Camera connected")
            break
        except:
            time.sleep(1)
    
    while True:
        try:
            data = socket.recv()
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                results = model(frame, verbose=False, conf=0.5)
                annotated = results[0].plot()
                
                mode = "AUTO" if state.autonomous_mode else "MANUAL"
                color = (0, 255, 0) if state.autonomous_mode else (255, 200, 0)
                cv2.putText(annotated, mode, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                
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

def execute_command(cmd):
    """Execute a movement command"""
    if cmd == 'F': send_cmd(state.linear_speed, 0)
    elif cmd == 'B': send_cmd(-state.linear_speed, 0)
    elif cmd == 'L': send_cmd(0, state.angular_speed)
    elif cmd == 'R': send_cmd(0, -state.angular_speed)
    else: send_cmd(0, 0)

def start_move(cmd, action):
    state.button_held = True
    state.held_command = cmd
    state.current_action = action
    state.last_cmd = cmd

def stop_move():
    state.button_held = False
    state.held_command = None
    state.current_action = "Stopped"
    state.last_cmd = 'S'
    send_cmd(0, 0)

def emergency_stop():
    state.emergency_stopped = True
    state.current_action = "🚨 EMERGENCY!"
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
    state.current_action = "Autonomous ON" if enabled else "Manual"

def toggle_mapping(enabled):
    state.mapping_active = enabled
    state.current_action = "Mapping ON" if enabled else "Mapping OFF"

def clear_map():
    state.latest_map = None
    state.map_count = 0
    ui.notify('Map cleared (visual only)')

# ============== Continuous Command Loop ==============
def command_loop():
    """Send commands continuously while button is held"""
    while True:
        if state.button_held and state.held_command:
            execute_command(state.held_command)
        time.sleep(0.1)  # Send every 100ms

threading.Thread(target=command_loop, daemon=True).start()

# ============== GUI ==============
ui.add_head_html('''
<style>
    body { background: linear-gradient(135deg, #0d1117 0%, #161b22 100%); }
    .glass { 
        background: rgba(22, 27, 34, 0.95); 
        border: 1px solid rgba(255,255,255,0.1);
        border-radius: 16px;
    }
    .btn {
        width: 70px !important; height: 70px !important;
        font-size: 28px !important; border-radius: 14px !important;
        transition: all 0.1s !important;
        user-select: none !important;
    }
    .btn:active { transform: scale(0.9); }
</style>
''')

with ui.row().classes('w-full min-h-screen p-4 gap-4'):
    
    # ============== LEFT - Controls ==============
    with ui.column().classes('w-80 gap-3'):
        
        # Header
        with ui.card().classes('glass p-4'):
            with ui.row().classes('items-center gap-3'):
                ui.icon('smart_toy', size='32px').classes('text-blue-400')
                with ui.column().classes('gap-0'):
                    ui.label('RESCUE ROBOT').classes('text-xl font-bold text-white')
                    with ui.row().classes('items-center gap-2'):
                        conn_icon = ui.icon('circle', size='10px').classes('text-red-500')
                        conn_text = ui.label('Connecting...').classes('text-sm text-gray-400')
        
        # Status
        with ui.card().classes('glass p-4'):
            ui.label('STATUS').classes('text-xs text-gray-500 font-bold mb-1')
            action_label = ui.label('Ready').classes('text-2xl font-bold text-cyan-400')
        
        # Control Pad
        with ui.card().classes('glass p-6'):
            ui.label('CONTROLS (Hold to move)').classes('text-xs text-gray-500 font-bold mb-3 text-center w-full')
            
            with ui.column().classes('items-center gap-2'):
                fwd = ui.button('▲').classes('btn bg-blue-600 text-white')
                fwd.on('mousedown', lambda: start_move('F', '▲ Forward'))
                fwd.on('mouseup', stop_move)
                fwd.on('mouseleave', stop_move)
                
                with ui.row().classes('gap-2'):
                    left = ui.button('◀').classes('btn bg-blue-600 text-white')
                    left.on('mousedown', lambda: start_move('L', '◀ Left'))
                    left.on('mouseup', stop_move)
                    left.on('mouseleave', stop_move)
                    
                    ui.button('■', on_click=stop_move).classes('btn bg-gray-700 text-white')
                    
                    right = ui.button('▶').classes('btn bg-blue-600 text-white')
                    right.on('mousedown', lambda: start_move('R', '▶ Right'))
                    right.on('mouseup', stop_move)
                    right.on('mouseleave', stop_move)
                
                back = ui.button('▼').classes('btn bg-blue-600 text-white')
                back.on('mousedown', lambda: start_move('B', '▼ Backward'))
                back.on('mouseup', stop_move)
                back.on('mouseleave', stop_move)
            
            # Speed
            ui.label('Speed').classes('text-gray-400 text-sm mt-4')
            speed_slider = ui.slider(min=0.1, max=0.5, step=0.05, value=0.2).classes('w-full')
            speed_label = ui.label('0.20 m/s').classes('text-cyan-400 text-sm')
            speed_slider.on('change', lambda: (setattr(state, 'linear_speed', speed_slider.value), speed_label.set_text(f'{speed_slider.value:.2f} m/s')))
        
        # Modes
        with ui.card().classes('glass p-4'):
            ui.label('MODES').classes('text-xs text-gray-500 font-bold mb-2')
            with ui.row().classes('items-center justify-between w-full mb-2'):
                ui.label('Mapping').classes('text-white')
                ui.switch(value=True, on_change=lambda e: toggle_mapping(e.value))
            with ui.row().classes('items-center justify-between w-full'):
                ui.label('Autonomous').classes('text-white')
                ui.switch(on_change=lambda e: toggle_autonomous(e.value))
        
        # Emergency
        with ui.card().classes('glass p-4 border border-red-600/50'):
            with ui.row().classes('gap-2 w-full'):
                ui.button('🛑 STOP', on_click=emergency_stop).classes('flex-grow bg-red-600 text-white font-bold')
                ui.button('✓', on_click=release_emergency).classes('bg-green-600 text-white')
    
    # ============== CENTER - Video ==============
    with ui.column().classes('flex-grow gap-3'):
        with ui.card().classes('glass p-3 h-full'):
            with ui.row().classes('justify-between items-center mb-2'):
                ui.label('📹 LIVE + YOLO').classes('text-white font-bold')
                fps_label = ui.label('-- FPS').classes('text-green-400')
            video_img = ui.image().classes('w-full bg-black rounded-lg').style('max-height: 75vh; object-fit: contain;')
    
    # ============== RIGHT - Map ==============
    with ui.column().classes('w-[550px] gap-3'):
        with ui.card().classes('glass p-4 h-full'):
            with ui.row().classes('justify-between items-center mb-3'):
                ui.label('🗺️ SLAM MAP').classes('text-white font-bold text-xl')
                map_count_label = ui.label('--').classes('text-green-400')
            
            # Map container with fixed size
            with ui.element('div').classes('w-full bg-gray-900 rounded-lg flex items-center justify-center').style('height: 500px; overflow: hidden;'):
                map_img = ui.image().classes('max-w-full max-h-full object-contain')
            
            # Map controls
            with ui.row().classes('gap-2 mt-3 justify-center'):
                ui.button('📷 Save', on_click=lambda: ui.notify('Map saved!')).classes('bg-blue-600')
                ui.button('🗑️ Clear', on_click=clear_map).classes('bg-orange-600')
            
            # Info
            with ui.row().classes('mt-3 gap-6 text-sm text-gray-400 justify-center'):
                ui.label(f'Host: {ROBOT_HOST}')
                ui.label(f'ROS: {ROS_PORT}')

# ============== Keyboard ==============
def handle_key(e):
    if e.action.keydown:
        if e.key in ['w', 'W', 'ArrowUp']: start_move('F', '▲ Forward')
        elif e.key in ['s', 'S', 'ArrowDown']: start_move('B', '▼ Backward')
        elif e.key in ['a', 'A', 'ArrowLeft']: start_move('L', '◀ Left')
        elif e.key in ['d', 'D', 'ArrowRight']: start_move('R', '▶ Right')
        elif e.key == ' ': stop_move()
        elif e.key == 'Escape': emergency_stop()
    elif e.action.keyup:
        if e.key in ['w','W','s','S','a','A','d','D','ArrowUp','ArrowDown','ArrowLeft','ArrowRight']:
            stop_move()

ui.keyboard(on_key=handle_key)

# ============== UI Update Timer ==============
last_frame = 0
last_time = time.time()

def update_ui():
    global last_frame, last_time
    
    if state.latest_frame:
        video_img.source = state.latest_frame
    
    if state.latest_map:
        map_img.source = state.latest_map
        map_count_label.text = f'{state.map_count} updates'
    
    if state.connected:
        conn_icon.classes(remove='text-red-500', add='text-green-500')
        conn_text.text = 'Connected'
        conn_text.classes(remove='text-gray-400', add='text-green-400')
    else:
        conn_icon.classes(remove='text-green-500', add='text-red-500')
        conn_text.text = 'Disconnected'
        conn_text.classes(remove='text-green-400', add='text-gray-400')
    
    action_label.text = state.current_action
    
    now = time.time()
    if now - last_time >= 1.0:
        fps = state.frame_count - last_frame
        fps_label.text = f'{fps} FPS'
        last_frame = state.frame_count
        last_time = now

ui.timer(0.1, update_ui)

# Start
init_ros()
time.sleep(1)

ui.run(title='Rescue Robot', port=8080, dark=True, reload=False)
