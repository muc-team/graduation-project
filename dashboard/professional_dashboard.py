#!/usr/bin/env python3
"""
Professional Robot Dashboard - Final Version
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

# Config
ROBOT_HOST = 'robot.local'
ROS_PORT = 9090
VIDEO_PORT = 5555

# State
class State:
    connected = False
    mapping_active = False  # Starts OFF
    autonomous = False
    emergency = False
    action = "Ready"
    speed = 0.2
    frame = None
    map_img = None
    frame_count = 0
    map_count = 0
    button_held = False
    held_cmd = None

state = State()

# ROS
client = None
manual_topic = None
estop_topic = None
explore_topic = None

def init_ros():
    global client, manual_topic, estop_topic, explore_topic
    client = roslibpy.Ros(host=ROBOT_HOST, port=ROS_PORT)
    
    def on_ready():
        global manual_topic, estop_topic, explore_topic
        state.connected = True
        print("✅ ROS Connected")
        
        manual_topic = roslibpy.Topic(client, '/manual_cmd', 'geometry_msgs/Twist')
        manual_topic.advertise()
        estop_topic = roslibpy.Topic(client, '/emergency_stop', 'std_msgs/Bool')
        estop_topic.advertise()
        explore_topic = roslibpy.Topic(client, '/explore_enable', 'std_msgs/Bool')
        explore_topic.advertise()
        
        roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid').subscribe(on_map)
    
    client.on_ready(on_ready)
    client.on('close', lambda e: setattr(state, 'connected', False))
    
    def run():
        while True:
            try:
                if not client.is_connected:
                    client.run()
            except: pass
            time.sleep(2)
    threading.Thread(target=run, daemon=True).start()

def on_map(msg):
    if not state.mapping_active:
        return
    try:
        w, h = msg['info']['width'], msg['info']['height']
        data = np.array(msg['data'], dtype=np.int8).reshape((h, w))
        
        # Professional color scheme
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[data == -1] = [35, 35, 45]       # Unknown - dark
        img[data == 0] = [240, 245, 250]     # Free - light
        img[data == 100] = [80, 80, 200]     # Wall - red
        
        img = np.flipud(img)
        
        # Scale
        scale = max(2, 600 // max(w, h))
        img = cv2.resize(img, (w*scale, h*scale), interpolation=cv2.INTER_NEAREST)
        
        # Robot marker
        cx, cy = img.shape[1]//2, img.shape[0]//2
        cv2.circle(img, (cx, cy), 15, (50, 200, 50), -1)
        cv2.circle(img, (cx, cy), 18, (100, 255, 100), 3)
        
        # Direction arrow
        cv2.arrowedLine(img, (cx, cy), (cx, cy-25), (100, 255, 100), 3, tipLength=0.4)
        
        _, buf = cv2.imencode('.png', img)
        state.map_img = f'data:image/png;base64,{base64.b64encode(buf).decode()}'
        state.map_count += 1
    except: pass

# Video
print("Loading YOLO...")
model = YOLO("../models/yolov8n.pt")
print("✅ YOLO Ready")

def video_loop():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt_string(zmq.SUBSCRIBE, '')
    sock.setsockopt(zmq.RCVTIMEO, 5000)
    
    while True:
        try:
            sock.connect(f"tcp://{ROBOT_HOST}:{VIDEO_PORT}")
            print("📷 Camera Ready")
            break
        except: time.sleep(1)
    
    while True:
        try:
            data = sock.recv()
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                results = model(frame, verbose=False, conf=0.5)
                annotated = results[0].plot()
                _, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
                state.frame = f'data:image/jpeg;base64,{base64.b64encode(buf).decode()}'
                state.frame_count += 1
        except: time.sleep(0.1)

threading.Thread(target=video_loop, daemon=True).start()

# Control
def send_cmd(l=0.0, a=0.0):
    if manual_topic and state.connected and not state.emergency:
        manual_topic.publish(roslibpy.Message({'linear':{'x':l,'y':0,'z':0},'angular':{'x':0,'y':0,'z':a}}))

def exec_cmd(cmd):
    if cmd == 'F': send_cmd(state.speed, 0)
    elif cmd == 'B': send_cmd(-state.speed, 0)
    elif cmd == 'L': send_cmd(0, 0.5)
    elif cmd == 'R': send_cmd(0, -0.5)
    else: send_cmd(0, 0)

def start(cmd, txt):
    state.button_held = True
    state.held_cmd = cmd
    state.action = txt

def stop():
    state.button_held = False
    state.held_cmd = None
    state.action = "Ready"
    send_cmd(0, 0)

def estop():
    state.emergency = True
    state.action = "🛑 EMERGENCY"
    if estop_topic: estop_topic.publish(roslibpy.Message({'data': True}))
    send_cmd(0, 0)

def release():
    state.emergency = False
    state.action = "Ready"
    if estop_topic: estop_topic.publish(roslibpy.Message({'data': False}))

def toggle_auto(v):
    state.autonomous = v
    if explore_topic: explore_topic.publish(roslibpy.Message({'data': v}))
    state.action = "AUTO ON" if v else "Manual"

def toggle_map(v):
    state.mapping_active = v
    state.action = "Mapping..." if v else "Mapping OFF"

def clear_map():
    state.map_img = None
    state.map_count = 0

# Command loop
def cmd_loop():
    while True:
        if state.button_held and state.held_cmd:
            exec_cmd(state.held_cmd)
        time.sleep(0.1)
threading.Thread(target=cmd_loop, daemon=True).start()

# GUI
ui.add_head_html('<style>body{background:#0d1117;}.card{background:rgba(22,27,34,0.95);border:1px solid rgba(255,255,255,0.1);border-radius:16px;}.btn{width:70px!important;height:70px!important;font-size:28px!important;border-radius:14px!important;}</style>')

with ui.row().classes('w-full min-h-screen p-4 gap-4'):
    
    # Left
    with ui.column().classes('w-72 gap-3'):
        with ui.card().classes('card p-4'):
            with ui.row().classes('items-center gap-2'):
                ui.icon('smart_toy', size='28px').classes('text-blue-400')
                ui.label('RESCUE ROBOT').classes('text-lg font-bold text-white')
            with ui.row().classes('items-center gap-2 mt-2'):
                conn_icon = ui.icon('circle', size='10px').classes('text-red-500')
                conn_text = ui.label('Connecting...').classes('text-sm text-gray-400')
        
        with ui.card().classes('card p-4'):
            action_label = ui.label('Ready').classes('text-2xl font-bold text-cyan-400')
        
        with ui.card().classes('card p-5'):
            ui.label('Hold to move').classes('text-xs text-gray-500 mb-3 text-center w-full')
            with ui.column().classes('items-center gap-1'):
                f = ui.button('▲').classes('btn bg-blue-600')
                f.on('mousedown', lambda: start('F', '▲')); f.on('mouseup', stop); f.on('mouseleave', stop)
                with ui.row().classes('gap-1'):
                    l = ui.button('◀').classes('btn bg-blue-600'); l.on('mousedown', lambda: start('L', '◀')); l.on('mouseup', stop); l.on('mouseleave', stop)
                    ui.button('■', on_click=stop).classes('btn bg-gray-700')
                    r = ui.button('▶').classes('btn bg-blue-600'); r.on('mousedown', lambda: start('R', '▶')); r.on('mouseup', stop); r.on('mouseleave', stop)
                b = ui.button('▼').classes('btn bg-blue-600'); b.on('mousedown', lambda: start('B', '▼')); b.on('mouseup', stop); b.on('mouseleave', stop)
            
            ui.label('Speed').classes('text-gray-400 text-sm mt-3')
            slider = ui.slider(min=0.1, max=0.5, step=0.05, value=0.2).classes('w-full')
            slider.on('change', lambda: setattr(state, 'speed', slider.value))
        
        with ui.card().classes('card p-3'):
            with ui.row().classes('justify-between items-center w-full'):
                ui.label('Mapping').classes('text-white')
                ui.switch(on_change=lambda e: toggle_map(e.value))
            with ui.row().classes('justify-between items-center w-full mt-2'):
                ui.label('Autonomous').classes('text-white')
                ui.switch(on_change=lambda e: toggle_auto(e.value))
        
        with ui.card().classes('card p-3 border border-red-600/50'):
            with ui.row().classes('gap-2 w-full'):
                ui.button('🛑 STOP', on_click=estop).classes('flex-grow bg-red-600 font-bold')
                ui.button('✓', on_click=release).classes('bg-green-600')
    
    # Center - Video
    with ui.column().classes('flex-grow gap-3'):
        with ui.card().classes('card p-3 h-full'):
            with ui.row().classes('justify-between mb-2'):
                ui.label('📹 LIVE').classes('text-white font-bold')
                fps = ui.label('--').classes('text-green-400')
            video = ui.image().classes('w-full rounded-lg').style('max-height:75vh;object-fit:contain;')
    
    # Right - Map
    with ui.column().classes('w-[600px] gap-3'):
        with ui.card().classes('card p-4 h-full'):
            with ui.row().classes('justify-between items-center mb-3'):
                ui.label('🗺️ SLAM MAP').classes('text-white font-bold text-xl')
                map_count = ui.label('OFF').classes('text-gray-400')
            
            with ui.element('div').classes('w-full bg-gray-900 rounded-xl flex items-center justify-center').style('height:550px;'):
                map_view = ui.image().classes('max-w-full max-h-full')
                no_map = ui.label('Enable Mapping to start').classes('text-gray-500 text-lg')
            
            with ui.row().classes('gap-3 mt-3 justify-center'):
                ui.button('🗑️ Clear', on_click=clear_map).classes('bg-orange-600')

# Keyboard
def key(e):
    if e.action.keydown:
        if e.key in ['w','W','ArrowUp']: start('F','▲')
        elif e.key in ['s','S','ArrowDown']: start('B','▼')
        elif e.key in ['a','A','ArrowLeft']: start('L','◀')
        elif e.key in ['d','D','ArrowRight']: start('R','▶')
        elif e.key == ' ': stop()
        elif e.key == 'Escape': estop()
    elif e.action.keyup:
        if e.key in ['w','W','s','S','a','A','d','D','ArrowUp','ArrowDown','ArrowLeft','ArrowRight']: stop()
ui.keyboard(on_key=key)

# Update
last_f, last_t = 0, time.time()
def update():
    global last_f, last_t
    if state.frame: video.source = state.frame
    if state.map_img:
        map_view.source = state.map_img
        map_count.text = f'{state.map_count} updates'
        map_count.classes(remove='text-gray-400', add='text-green-400')
        no_map.set_visibility(False)
    else:
        no_map.set_visibility(True)
        map_count.text = 'OFF' if not state.mapping_active else 'Starting...'
    
    if state.connected:
        conn_icon.classes(remove='text-red-500', add='text-green-500')
        conn_text.text = 'Connected'
    else:
        conn_icon.classes(remove='text-green-500', add='text-red-500')
        conn_text.text = 'Disconnected'
    
    action_label.text = state.action
    
    now = time.time()
    if now - last_t >= 1:
        fps.text = f'{state.frame_count - last_f} FPS'
        last_f, last_t = state.frame_count, now

ui.timer(0.1, update)

init_ros()
time.sleep(1)
ui.run(title='Rescue Robot', port=8080, dark=True, reload=False)
