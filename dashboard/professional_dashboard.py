#!/usr/bin/env python3
"""
RESCUE ROBOT - Professional Dashboard
Modern UI with Tailwind-like styling
"""

import cv2, zmq, time, base64, roslibpy, threading, numpy as np, subprocess
from nicegui import ui, app as nicegui_app
from ultralytics import YOLO

ROBOT = 'robot.local'

class State:
    ros_connected = False
    mapping_on = False
    auto_on = False
    emergency = False
    action = "Ready"
    speed = 0.15
    video_frame = None
    map_frame = None
    fps = 0
    held_key = None

S = State()
ros_client = None
topic_manual = topic_estop = topic_explore = topic_slam_reset = None


def ros_connect():
    global ros_client, topic_manual, topic_estop, topic_explore
    ros_client = roslibpy.Ros(host=ROBOT, port=9090)
    
    def on_ready():
        global topic_manual, topic_estop, topic_explore
        S.ros_connected = True
        topic_manual = roslibpy.Topic(ros_client, '/manual_cmd', 'geometry_msgs/Twist')
        topic_manual.advertise()
        topic_estop = roslibpy.Topic(ros_client, '/emergency_stop', 'std_msgs/Bool')
        topic_estop.advertise()
        topic_explore = roslibpy.Topic(ros_client, '/explore_enable', 'std_msgs/Bool')
        explore.advertise()
        roslibpy.Topic(ros_client, '/map', 'nav_msgs/OccupancyGrid').subscribe(on_map)
        print("✅ ROS Connected")
    
    ros_client.on_ready(on_ready)
    ros_client.on('close', lambda e: setattr(S, 'ros_connected', False))
    
    def reconnect_loop():
        while True:
            try:
                if not ros_client.is_connected:
                    ros_client.run()
            except: pass
            time.sleep(3)
    threading.Thread(target=reconnect_loop, daemon=True).start()


def on_map(msg):
    if not S.mapping_on:
        return
    try:
        w = msg['info']['width']
        h = msg['info']['height']
        data = np.array(msg['data'], dtype=np.int8).reshape((h, w))
        
        # Create RGB image
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[data == -1] = [25, 25, 35]       # Unknown = dark gray
        img[data == 0] = [230, 235, 240]     # Free = off-white
        img[data == 100] = [60, 60, 160]     # Obstacle = dark red
        
        img = np.flipud(img)
        
        # Scale for display
        scale = max(2, 500 // max(w, h))
        img = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
        
        # Draw robot at center
        cx, cy = img.shape[1] // 2, img.shape[0] // 2
        
        # Robot body
        cv2.rectangle(img, (cx-20, cy-25), (cx+20, cy+25), (0, 150, 0), -1)
        cv2.rectangle(img, (cx-20, cy-25), (cx+20, cy+25), (0, 220, 0), 2)
        
        # Direction indicator
        pts = np.array([[cx, cy-35], [cx-12, cy-20], [cx+12, cy-20]], np.int32)
        cv2.fillPoly(img, [pts], (0, 255, 255))
        
        _, buf = cv2.imencode('.png', img)
        S.map_frame = f'data:image/png;base64,{base64.b64encode(buf).decode()}'
    except Exception as e:
        print(f"Map error: {e}")


# YOLO
print("Loading YOLO model...")
yolo_model = YOLO("../models/yolov8n.pt")
print("✅ YOLO Ready")


def video_thread():
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
    
    frame_count = 0
    last_time = time.time()
    
    while True:
        try:
            data = sock.recv()
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                results = yolo_model(frame, verbose=False, conf=0.5)
                annotated = results[0].plot()
                _, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
                S.video_frame = f'data:image/jpeg;base64,{base64.b64encode(buf).decode()}'
                frame_count += 1
                
                now = time.time()
                if now - last_time >= 1.0:
                    S.fps = frame_count
                    frame_count = 0
                    last_time = now
        except:
            time.sleep(0.05)

threading.Thread(target=video_thread, daemon=True).start()


# Controls
def send_twist(lin=0.0, ang=0.0):
    if topic_manual and S.ros_connected and not S.emergency:
        topic_manual.publish(roslibpy.Message({
            'linear': {'x': lin, 'y': 0, 'z': 0},
            'angular': {'x': 0, 'y': 0, 'z': ang}
        }))


def execute_held():
    cmd = S.held_key
    if cmd == 'F': send_twist(S.speed, 0)
    elif cmd == 'B': send_twist(-S.speed, 0)
    elif cmd == 'L': send_twist(0, 0.5)
    elif cmd == 'R': send_twist(0, -0.5)


def key_down(cmd, label):
    S.held_key = cmd
    S.action = label


def key_up():
    S.held_key = None
    S.action = "Stopped"
    send_twist(0, 0)


def emergency_stop():
    S.emergency = True
    S.action = "🛑 EMERGENCY"
    if topic_estop:
        topic_estop.publish(roslibpy.Message({'data': True}))
    send_twist(0, 0)


def emergency_release():
    S.emergency = False
    S.action = "Ready"
    if topic_estop:
        topic_estop.publish(roslibpy.Message({'data': False}))


def toggle_mapping(val):
    S.mapping_on = val
    if not val:
        S.map_frame = None
    S.action = "📍 Mapping ON" if val else "Mapping OFF"


def toggle_auto(val):
    S.auto_on = val
    if topic_explore:
        topic_explore.publish(roslibpy.Message({'data': val}))
    S.action = "🤖 AUTO MODE" if val else "Manual"


def clear_map():
    S.map_frame = None
    ui.notify('Map display cleared', type='info')


# Continuous command sender
def cmd_thread():
    while True:
        if S.held_key:
            execute_held()
        time.sleep(0.08)

threading.Thread(target=cmd_thread, daemon=True).start()


# =========== GUI ===========
ui.add_head_html('''
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
    * { font-family: 'Inter', sans-serif; }
    body { 
        background: linear-gradient(145deg, #0c0f1a 0%, #1a1f35 50%, #0c0f1a 100%);
        min-height: 100vh;
    }
    .glass-panel {
        background: rgba(15, 20, 35, 0.85);
        backdrop-filter: blur(12px);
        border: 1px solid rgba(100, 120, 180, 0.25);
        border-radius: 16px;
        box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);
    }
    .control-btn {
        width: 65px !important;
        height: 65px !important;
        font-size: 26px !important;
        border-radius: 14px !important;
        transition: all 0.15s ease !important;
        background: linear-gradient(135deg, #3b82f6, #2563eb) !important;
    }
    .control-btn:hover { transform: scale(1.05); box-shadow: 0 4px 20px rgba(59, 130, 246, 0.5); }
    .control-btn:active { transform: scale(0.92); }
    .stop-btn { background: linear-gradient(135deg, #475569, #334155) !important; }
    .status-glow {
        animation: glow 2s ease-in-out infinite alternate;
    }
    @keyframes glow {
        from { box-shadow: 0 0 5px #22c55e; }
        to { box-shadow: 0 0 20px #22c55e; }
    }
</style>
''')

with ui.row().classes('w-full min-h-screen p-5 gap-5'):
    
    # === LEFT PANEL ===
    with ui.column().classes('gap-4').style('width: 300px;'):
        
        # Header
        with ui.card().classes('glass-panel p-5'):
            with ui.row().classes('items-center gap-3'):
                ui.icon('precision_manufacturing', size='32px').classes('text-blue-400')
                ui.label('RESCUE ROBOT').classes('text-xl font-bold text-white tracking-wide')
            with ui.row().classes('items-center gap-2 mt-3'):
                status_dot = ui.element('div').classes('w-3 h-3 rounded-full bg-red-500')
                status_label = ui.label('Connecting...').classes('text-sm text-slate-400')
        
        # Status Card
        with ui.card().classes('glass-panel p-4'):
            ui.label('CURRENT ACTION').classes('text-xs text-slate-500 font-semibold tracking-wider')
            action_display = ui.label('Ready').classes('text-2xl font-bold text-cyan-400 mt-1')
        
        # Control Pad
        with ui.card().classes('glass-panel p-5'):
            ui.label('MANUAL CONTROL').classes('text-xs text-slate-500 font-semibold tracking-wider mb-4 text-center w-full')
            
            with ui.column().classes('items-center gap-2'):
                btn_f = ui.button('▲').classes('control-btn text-white')
                btn_f.on('mousedown', lambda: key_down('F', '▲ FORWARD'))
                btn_f.on('mouseup', key_up).on('mouseleave', key_up)
                
                with ui.row().classes('gap-2'):
                    btn_l = ui.button('◀').classes('control-btn text-white')
                    btn_l.on('mousedown', lambda: key_down('L', '◀ LEFT'))
                    btn_l.on('mouseup', key_up).on('mouseleave', key_up)
                    
                    ui.button('■', on_click=key_up).classes('control-btn stop-btn text-white')
                    
                    btn_r = ui.button('▶').classes('control-btn text-white')
                    btn_r.on('mousedown', lambda: key_down('R', '▶ RIGHT'))
                    btn_r.on('mouseup', key_up).on('mouseleave', key_up)
                
                btn_b = ui.button('▼').classes('control-btn text-white')
                btn_b.on('mousedown', lambda: key_down('B', '▼ BACK'))
                btn_b.on('mouseup', key_up).on('mouseleave', key_up)
            
            # Speed slider
            with ui.row().classes('items-center gap-3 mt-5 w-full'):
                ui.icon('speed', size='20px').classes('text-slate-400')
                speed_slider = ui.slider(min=0.1, max=0.3, step=0.02, value=0.15).classes('flex-grow')
                speed_value = ui.label('0.15').classes('text-cyan-400 font-mono w-12')
            
            speed_slider.on('change', lambda: (setattr(S, 'speed', speed_slider.value), speed_value.set_text(f'{speed_slider.value:.2f}')))
        
        # Toggles
        with ui.card().classes('glass-panel p-4'):
            with ui.row().classes('justify-between items-center w-full'):
                with ui.row().classes('items-center gap-2'):
                    ui.icon('map', size='20px').classes('text-blue-400')
                    ui.label('Mapping').classes('text-white font-medium')
                ui.switch(on_change=lambda e: toggle_mapping(e.value)).classes('scale-110')
            
            ui.separator().classes('my-3 bg-slate-700')
            
            with ui.row().classes('justify-between items-center w-full'):
                with ui.row().classes('items-center gap-2'):
                    ui.icon('smart_toy', size='20px').classes('text-green-400')
                    ui.label('Autonomous').classes('text-white font-medium')
                ui.switch(on_change=lambda e: toggle_auto(e.value)).classes('scale-110')
        
        # Emergency
        with ui.card().classes('glass-panel p-4 border-2 border-red-500/40'):
            ui.label('EMERGENCY CONTROLS').classes('text-xs text-red-400 font-semibold tracking-wider mb-3')
            with ui.row().classes('gap-3 w-full'):
                ui.button('🛑 STOP ALL', on_click=emergency_stop).classes('flex-grow bg-red-600 hover:bg-red-500 text-white font-bold py-3 rounded-xl')
                ui.button('✓', on_click=emergency_release).classes('bg-green-600 hover:bg-green-500 text-white font-bold px-4 py-3 rounded-xl')
    
    # === CENTER: VIDEO ===
    with ui.column().classes('flex-grow gap-4'):
        with ui.card().classes('glass-panel p-4 h-full'):
            with ui.row().classes('justify-between items-center mb-3'):
                with ui.row().classes('items-center gap-2'):
                    ui.icon('videocam', size='24px').classes('text-green-400')
                    ui.label('LIVE CAMERA + YOLO').classes('text-white font-bold')
                fps_display = ui.label('-- FPS').classes('text-green-400 font-mono bg-slate-800 px-3 py-1 rounded-lg')
            
            video_element = ui.image().classes('w-full rounded-xl bg-slate-900').style('max-height: 65vh; object-fit: contain;')
    
    # === RIGHT: MAP ===
    with ui.column().classes('gap-4').style('width: 550px;'):
        with ui.card().classes('glass-panel p-4 h-full'):
            with ui.row().classes('justify-between items-center mb-3'):
                with ui.row().classes('items-center gap-2'):
                    ui.icon('explore', size='24px').classes('text-yellow-400')
                    ui.label('SLAM MAP').classes('text-white font-bold text-lg')
                map_status = ui.label('OFF').classes('text-slate-400 font-mono bg-slate-800 px-3 py-1 rounded-lg')
            
            # Map container
            with ui.element('div').classes('w-full bg-slate-900/50 rounded-xl flex items-center justify-center relative overflow-hidden').style('height: calc(100vh - 220px); min-height: 450px;'):
                map_element = ui.image().classes('max-w-full max-h-full object-contain')
                no_map_text = ui.label('Enable Mapping to start scanning').classes('text-slate-500 text-lg absolute')
            
            # Map buttons
            with ui.row().classes('gap-3 mt-4 justify-center'):
                ui.button('🗑️ Clear Display', on_click=clear_map).classes('bg-orange-600 hover:bg-orange-500 text-white px-5 py-2 rounded-xl')


# Keyboard handler
def keyboard_handler(e):
    if e.action.keydown:
        if e.key in ['w', 'W', 'ArrowUp']: key_down('F', '▲ FORWARD')
        elif e.key in ['s', 'S', 'ArrowDown']: key_down('B', '▼ BACK')
        elif e.key in ['a', 'A', 'ArrowLeft']: key_down('L', '◀ LEFT')
        elif e.key in ['d', 'D', 'ArrowRight']: key_down('R', '▶ RIGHT')
        elif e.key == ' ': key_up()
        elif e.key == 'Escape': emergency_stop()
    elif e.action.keyup:
        if e.key in ['w', 'W', 's', 'S', 'a', 'A', 'd', 'D', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']:
            key_up()

ui.keyboard(on_key=keyboard_handler)


# UI Update timer
def update_ui():
    # Video
    if S.video_frame:
        video_element.source = S.video_frame
    
    # Map
    if S.map_frame:
        map_element.source = S.map_frame
        map_element.set_visibility(True)
        no_map_text.set_visibility(False)
        map_status.text = '● LIVE'
        map_status.classes(remove='text-slate-400', add='text-green-400')
    else:
        map_element.set_visibility(False)
        no_map_text.set_visibility(True)
        if S.mapping_on:
            map_status.text = 'Waiting...'
        else:
            map_status.text = 'OFF'
        map_status.classes(remove='text-green-400', add='text-slate-400')
    
    # Connection status
    if S.ros_connected:
        status_dot.classes(remove='bg-red-500', add='bg-green-500 status-glow')
        status_label.text = 'Connected'
        status_label.classes(remove='text-slate-400', add='text-green-400')
    else:
        status_dot.classes(remove='bg-green-500 status-glow', add='bg-red-500')
        status_label.text = 'Disconnected'
        status_label.classes(remove='text-green-400', add='text-slate-400')
    
    # Action
    action_display.text = S.action
    
    # FPS
    fps_display.text = f'{S.fps} FPS'


ui.timer(0.1, update_ui)

# Start
ros_connect()
ui.run(title='Rescue Robot Control', port=8080, dark=True, reload=False)
