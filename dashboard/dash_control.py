#!/usr/bin/env python3
"""
Robot Control Dashboard for PC
Connects to Raspberry Pi via WebSocket (roslibpy)
Handles: Video + YOLO, Map, Manual Control, Autonomous Toggle

Run on PC: python dash_control.py
"""

import cv2
import zmq
import time
import base64
import roslibpy
import threading
import numpy as np
from nicegui import ui
from ultralytics import YOLO

# ============== Configuration ==============
RASPBERRY_IP = 'robot.local'  # Using mDNS hostname
ROS_PORT = 9090
VIDEO_PORT = 5555

# ============== YOLO Model ==============
print("Loading YOLO model...")
model = YOLO("../models/yolov8n.pt")
print("YOLO loaded!")

# ============== ROS Connection ==============
client = roslibpy.Ros(host=RASPBERRY_IP, port=ROS_PORT)
connected = False

# Topics
manual_topic = None
estop_topic = None
explore_topic = None

# ============== State ==============
latest_frame = None
latest_map = None
frame_counter = 0
map_counter = 0
motor_status = "IDLE"
ros_connected = False

# ============== ROS Callbacks ==============
def map_callback(msg):
    global latest_map, map_counter
    try:
        width = msg['info']['width']
        height = msg['info']['height']
        data = np.array(msg['data'], dtype=np.int8).reshape((height, width))
        
        img = np.full((height, width, 3), 30, dtype=np.uint8)
        img[data == 0] = [255, 255, 255]
        img[data == 100] = [50, 50, 255]
        img = np.flipud(img)
        
        _, buffer = cv2.imencode('.png', img)
        latest_map = f'data:image/png;base64,{base64.b64encode(buffer).decode()}'
        map_counter += 1
    except:
        pass


def status_callback(msg):
    global motor_status
    motor_status = msg.get('data', 'IDLE')


def on_ros_ready():
    global manual_topic, estop_topic, explore_topic, ros_connected
    print("✅ Connected to ROS!")
    ros_connected = True
    
    # Subscribe to map
    map_listener = roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
    map_listener.subscribe(map_callback)
    
    # Subscribe to status
    status_listener = roslibpy.Topic(client, '/motor_status', 'std_msgs/String')
    status_listener.subscribe(status_callback)
    
    # Publishers
    manual_topic = roslibpy.Topic(client, '/manual_cmd', 'geometry_msgs/Twist')
    manual_topic.advertise()
    
    estop_topic = roslibpy.Topic(client, '/emergency_stop', 'std_msgs/Bool')
    estop_topic.advertise()
    
    explore_topic = roslibpy.Topic(client, '/explore_enable', 'std_msgs/Bool')
    explore_topic.advertise()


# ============== Video Stream ==============
def video_stream_loop():
    global latest_frame, frame_counter
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.setsockopt_string(zmq.SUBSCRIBE, '')
    socket.setsockopt(zmq.RCVTIMEO, 5000)
    
    while True:
        try:
            socket.connect(f"tcp://{RASPBERRY_IP}:{VIDEO_PORT}")
            print(f"📷 Connected to camera stream at {RASPBERRY_IP}:{VIDEO_PORT}")
            break
        except:
            time.sleep(1)
    
    while True:
        try:
            data = socket.recv()
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                # Run YOLO on PC
                results = model(frame, verbose=False)
                annotated = results[0].plot()
                
                _, buffer = cv2.imencode('.jpg', annotated)
                latest_frame = f'data:image/jpeg;base64,{base64.b64encode(buffer).decode()}'
                frame_counter += 1
        except:
            continue


# ============== ROS Connection Thread ==============
def ros_connection_thread():
    global ros_connected
    while True:
        try:
            if not client.is_connected:
                ros_connected = False
                client.run(timeout=5)
                if client.is_connected:
                    on_ros_ready()
        except:
            ros_connected = False
        time.sleep(2)


# Start threads
threading.Thread(target=ros_connection_thread, daemon=True).start()
threading.Thread(target=video_stream_loop, daemon=True).start()
time.sleep(2)

# ============== Control Functions ==============
LINEAR = 0.2
ANGULAR = 0.5
current_cmd = 'S'


def send_manual(linear=0.0, angular=0.0):
    if manual_topic:
        msg = {'linear': {'x': linear, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': angular}}
        manual_topic.publish(roslibpy.Message(msg))


def move(cmd, action):
    global current_cmd
    if current_cmd != cmd:
        current_cmd = cmd
        action_label.text = action
        
        if cmd == 'F':
            send_manual(LINEAR, 0)
        elif cmd == 'B':
            send_manual(-LINEAR, 0)
        elif cmd == 'L':
            send_manual(0, ANGULAR)
        elif cmd == 'R':
            send_manual(0, -ANGULAR)
        else:
            send_manual(0, 0)


def stop():
    move('S', '🛑 Stopped')


def emergency_stop():
    if estop_topic:
        estop_topic.publish(roslibpy.Message({'data': True}))
    action_label.text = '🚨 EMERGENCY STOP!'


def release_estop():
    if estop_topic:
        estop_topic.publish(roslibpy.Message({'data': False}))
    action_label.text = 'Ready'


def toggle_explore(enable):
    if explore_topic:
        explore_topic.publish(roslibpy.Message({'data': enable}))


# ============== GUI ==============
ui.add_head_html('''
<style>
    body { background: linear-gradient(135deg, #0a0a1a 0%, #1a1a3a 100%); }
    .control-btn {
        width: 70px !important; height: 70px !important;
        font-size: 28px !important; border-radius: 14px !important;
    }
    .control-btn:active { transform: scale(0.9); }
    .stop-btn {
        width: 70px !important; height: 70px !important;
        font-size: 16px !important; border-radius: 50% !important;
    }
</style>
''')

with ui.row().classes('w-full h-screen p-4 gap-4'):
    
    # Left: Video
    with ui.column().classes('w-1/2 gap-4'):
        with ui.card().classes('bg-slate-800 p-2 flex-grow'):
            with ui.row().classes('justify-between items-center mb-2'):
                ui.label('📹 LIVE FEED + YOLO').classes('text-white font-bold')
                fps_label = ui.label('--').classes('text-green-400')
            video_img = ui.image().classes('w-full h-96 bg-black rounded object-contain')
            video_img.source = ''
        
        # Controls
        with ui.card().classes('bg-slate-800 p-4'):
            with ui.row().classes('gap-8 items-start'):
                # Manual Control Pad
                with ui.column().classes('items-center'):
                    ui.label('Manual Control').classes('text-gray-400 text-sm mb-2')
                    action_label = ui.label('Ready').classes('text-xl font-bold text-green-400 mb-2')
                    
                    fwd = ui.button('⬆️').classes('control-btn bg-blue-600')
                    fwd.on('mousedown', lambda: move('F', '⬆️ Forward'))
                    fwd.on('mouseup', stop)
                    
                    with ui.row().classes('gap-1 my-1'):
                        left = ui.button('⬅️').classes('control-btn bg-blue-600')
                        left.on('mousedown', lambda: move('L', '⬅️ Left'))
                        left.on('mouseup', stop)
                        
                        ui.button('■').classes('stop-btn bg-gray-600').on('click', stop)
                        
                        right = ui.button('➡️').classes('control-btn bg-blue-600')
                        right.on('mousedown', lambda: move('R', '➡️ Right'))
                        right.on('mouseup', stop)
                    
                    back = ui.button('⬇️').classes('control-btn bg-blue-600')
                    back.on('mousedown', lambda: move('B', '⬇️ Backward'))
                    back.on('mouseup', stop)
                
                # Mode & Emergency
                with ui.column().classes('gap-4'):
                    with ui.card().classes('bg-slate-700 p-4'):
                        ui.label('Mode').classes('text-gray-400 text-sm')
                        auto_switch = ui.switch('Autonomous').classes('text-white')
                        auto_switch.on('change', lambda e: toggle_explore(e.value))
                    
                    with ui.card().classes('bg-red-900/50 p-4 border border-red-600'):
                        ui.label('Emergency').classes('text-red-400 text-sm')
                        with ui.row().classes('gap-2'):
                            ui.button('STOP', on_click=emergency_stop).classes('bg-red-600')
                            ui.button('Release', on_click=release_estop).classes('bg-green-600')
    
    # Right: Map
    with ui.column().classes('w-1/2 gap-4'):
        with ui.card().classes('bg-slate-800 p-4 flex-grow'):
            with ui.row().classes('justify-between items-center mb-2'):
                ui.label('🗺️ SLAM MAP').classes('text-white font-bold text-xl')
                conn_label = ui.label('Connecting...').classes('text-yellow-400')
            map_img = ui.image().classes('w-full h-full bg-gray-900 rounded object-contain')
            map_img.source = ''
        
        # Status
        with ui.card().classes('bg-slate-800 p-4'):
            ui.label('📊 Status').classes('text-white font-bold mb-2')
            with ui.row().classes('gap-8'):
                with ui.column():
                    ui.label('Robot IP').classes('text-gray-400 text-sm')
                    ui.label(RASPBERRY_IP).classes('text-cyan-400 font-mono')
                with ui.column():
                    ui.label('ROS Port').classes('text-gray-400 text-sm')
                    ui.label(str(ROS_PORT)).classes('text-cyan-400 font-mono')
                with ui.column():
                    ui.label('Video Port').classes('text-gray-400 text-sm')
                    ui.label(str(VIDEO_PORT)).classes('text-cyan-400 font-mono')


# Keyboard
def handle_key(e):
    if e.action.keydown:
        if e.key in ['w', 'W', 'ArrowUp']: move('F', '⬆️ Forward')
        elif e.key in ['s', 'S', 'ArrowDown']: move('B', '⬇️ Backward')
        elif e.key in ['a', 'A', 'ArrowLeft']: move('L', '⬅️ Left')
        elif e.key in ['d', 'D', 'ArrowRight']: move('R', '➡️ Right')
        elif e.key == ' ': stop()
        elif e.key == 'Escape': emergency_stop()
    elif e.action.keyup:
        if e.key in ['w','W','s','S','a','A','d','D','ArrowUp','ArrowDown','ArrowLeft','ArrowRight']:
            stop()

ui.keyboard(on_key=handle_key)


# Update UI
last_frame_count = 0
last_time = time.time()

def update_ui():
    global last_frame_count, last_time
    
    # Video
    if latest_frame:
        video_img.source = latest_frame
    
    # Map
    if latest_map:
        map_img.source = latest_map
    
    # Connection status
    if ros_connected:
        conn_label.text = '🟢 Connected'
        conn_label.classes(remove='text-yellow-400 text-red-400', add='text-green-400')
    else:
        conn_label.text = '🔴 Disconnected'
        conn_label.classes(remove='text-yellow-400 text-green-400', add='text-red-400')
    
    # FPS
    now = time.time()
    if now - last_time >= 1.0:
        fps = frame_counter - last_frame_count
        fps_label.text = f'{fps} FPS'
        last_frame_count = frame_counter
        last_time = now

ui.timer(0.1, update_ui)


ui.run(title='Robot Control - PC', port=8080, dark=True, reload=False)
