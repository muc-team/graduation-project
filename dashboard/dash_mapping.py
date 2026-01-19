import cv2
import zmq
import time
import base64
import roslibpy
import threading
import numpy as np
from nicegui import ui, app
from ultralytics import YOLO

RASPBERRY_IP = 'robot.local'
ROS_PORT, TCP_PORT = 9090, 5555

model = YOLO("../models/yolov8n.pt") 
client = roslibpy.Ros(host=RASPBERRY_IP, port=ROS_PORT)

# UI Elements
status_label = None
video_image = None
map_image = None
battery_chart = None
gas_knob = None
battery_knob = None
confidence_knob = None
log_container = None
connection_notified = False
speed_label = None
action_label = None

latest_frame_b64 = None 
latest_map_b64 = None 

frame_counter = 0        
ui_frame_counter = 0     
map_counter = 0          
ui_map_counter = 0

# Manual Control State
current_speed = 0.15
held_key = None
emergency_stopped = False
autonomous_mode = False

# ROS Publishers (initialize after connection)
manual_topic = None
estop_topic = None
explore_topic = None

def setup_publishers():
    """Setup ROS publishers and additional subscribers after connection."""
    global manual_topic, estop_topic, explore_topic, odom_listener, scan_listener
    if client.is_connected:
        manual_topic = roslibpy.Topic(client, '/manual_cmd', 'geometry_msgs/Twist')
        manual_topic.advertise()
        
        estop_topic = roslibpy.Topic(client, '/emergency_stop', 'std_msgs/Bool')
        estop_topic.advertise()
        
        explore_topic = roslibpy.Topic(client, '/explore_enable', 'std_msgs/Bool')
        explore_topic.advertise()
        
        # Subscribe to odom and scan for RViz-style map
        odom_listener = roslibpy.Topic(client, '/odom', 'nav_msgs/Odometry')
        odom_listener.subscribe(pose_callback)
        
        scan_listener = roslibpy.Topic(client, '/scan', 'sensor_msgs/LaserScan')
        scan_listener.subscribe(scan_callback)
        
        # Start render timer
        start_map_render_timer()
        
        print("✅ ROS Publishers and subscribers ready")

def send_twist(linear: float, angular: float):
    """Send velocity command to robot."""
    global manual_topic
    if manual_topic and client.is_connected and not emergency_stopped:
        manual_topic.publish(roslibpy.Message({
            'linear': {'x': linear, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': angular}
        }))

def move_forward():
    global held_key
    held_key = 'F'
    send_twist(current_speed, 0)
    update_action("Moving Forward")

def move_backward():
    global held_key
    held_key = 'B'
    send_twist(-current_speed, 0)
    update_action("Moving Backward")

def turn_left():
    global held_key
    held_key = 'L'
    send_twist(0, 0.5)
    update_action("Turning Left")

def turn_right():
    global held_key
    held_key = 'R'
    send_twist(0, -0.5)
    update_action("Turning Right")

def stop_robot():
    global held_key
    held_key = None
    send_twist(0, 0)
    update_action("Stopped")

def emergency_stop():
    global emergency_stopped, held_key
    emergency_stopped = True
    held_key = None
    send_twist(0, 0)
    if estop_topic and client.is_connected:
        estop_topic.publish(roslibpy.Message({'data': True}))
    update_action("🛑 EMERGENCY STOP")
    ui.notify('EMERGENCY STOP ACTIVATED!', type='negative')

def release_emergency():
    global emergency_stopped
    emergency_stopped = False
    if estop_topic and client.is_connected:
        estop_topic.publish(roslibpy.Message({'data': False}))
    update_action("Ready")
    ui.notify('Emergency released', type='positive')

def toggle_autonomous(enabled: bool):
    global autonomous_mode
    autonomous_mode = enabled
    if explore_topic and client.is_connected:
        explore_topic.publish(roslibpy.Message({'data': enabled}))
    update_action("AUTONOMOUS" if enabled else "Manual")
    ui.notify(f"Autonomous mode {'enabled' if enabled else 'disabled'}", type='info')

def update_speed(value: float):
    global current_speed
    current_speed = value
    if speed_label:
        speed_label.text = f'{value:.2f} m/s'

def update_action(text: str):
    if action_label:
        action_label.text = text       

# Map data storage for RViz-style rendering
map_info = {'width': 0, 'height': 0, 'resolution': 0.05, 'origin_x': 0, 'origin_y': 0, 'data': None}
robot_pose = {'x': 0, 'y': 0, 'theta': 0}
laser_points = []

def map_callback(msg):
    """Store map data for rendering."""
    global map_info
    try:
        map_info['width'] = msg['info']['width']
        map_info['height'] = msg['info']['height']
        map_info['resolution'] = msg['info']['resolution']
        map_info['origin_x'] = msg['info']['origin']['position']['x']
        map_info['origin_y'] = msg['info']['origin']['position']['y']
        map_info['data'] = np.array(msg['data'], dtype=np.int8)
    except Exception as e:
        pass

def pose_callback(msg):
    """Get robot pose from odometry."""
    global robot_pose
    try:
        robot_pose['x'] = msg['pose']['pose']['position']['x']
        robot_pose['y'] = msg['pose']['pose']['position']['y']
        # Extract yaw from quaternion
        q = msg['pose']['pose']['orientation']
        siny = 2.0 * (q['w'] * q['z'] + q['x'] * q['y'])
        cosy = 1.0 - 2.0 * (q['y'] * q['y'] + q['z'] * q['z'])
        robot_pose['theta'] = np.arctan2(siny, cosy)
    except Exception as e:
        pass

def scan_callback(msg):
    """Get laser scan points for overlay."""
    global laser_points
    try:
        ranges = msg['ranges']
        angle_min = msg['angle_min']
        angle_increment = msg['angle_increment']
        
        points = []
        for i, r in enumerate(ranges):
            if r > 0.05 and r < 12.0 and np.isfinite(r):
                angle = angle_min + i * angle_increment + robot_pose['theta']
                px = robot_pose['x'] + r * np.cos(angle)
                py = robot_pose['y'] + r * np.sin(angle)
                points.append((px, py))
        laser_points = points
    except Exception as e:
        pass

def render_rviz_map():
    """Render map exactly like RViz with robot and laser scan."""
    global latest_map_b64, map_counter
    
    if map_info['data'] is None or map_info['width'] == 0:
        return
    
    try:
        w = map_info['width']
        h = map_info['height']
        res = map_info['resolution']
        origin_x = map_info['origin_x']
        origin_y = map_info['origin_y']
        
        data = map_info['data'].reshape((h, w))
        
        # RViz exact colors (BGR for OpenCV)
        img = np.full((h, w, 3), 205, dtype=np.uint8)  # Unknown = gray #CDCDCD
        img[data == 0] = [254, 254, 254]    # Free = almost white
        img[data == 100] = [0, 0, 0]        # Occupied = black
        
        # Scale up for better visibility
        scale = max(2, min(4, 600 // max(w, h)))
        img = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
        
        # Convert robot position to pixel coordinates
        rx = int((robot_pose['x'] - origin_x) / res * scale)
        ry = int((h - (robot_pose['y'] - origin_y) / res) * scale)  # Flip Y
        
        # Draw laser scan points (red dots like RViz)
        for px, py in laser_points:
            lx = int((px - origin_x) / res * scale)
            ly = int((h - (py - origin_y) / res) * scale)
            if 0 <= lx < w * scale and 0 <= ly < h * scale:
                cv2.circle(img, (lx, ly), max(1, scale // 2), (0, 0, 255), -1)
        
        # Draw robot (green rectangle with direction arrow - like RViz)
        robot_size = int(0.23 / res * scale)  # 23cm robot radius
        if 0 <= rx < w * scale and 0 <= ry < h * scale:
            # Robot body (filled green circle like RViz default)
            cv2.circle(img, (rx, ry), robot_size, (0, 180, 0), -1)
            cv2.circle(img, (rx, ry), robot_size, (0, 255, 0), 2)
            
            # Direction arrow
            arrow_len = robot_size + int(10 * scale / 3)
            ax = int(rx + arrow_len * np.cos(-robot_pose['theta']))
            ay = int(ry + arrow_len * np.sin(-robot_pose['theta']))
            cv2.arrowedLine(img, (rx, ry), (ax, ay), (0, 255, 255), max(2, scale), tipLength=0.4)
        
        # Encode to base64
        _, buffer = cv2.imencode('.png', img)
        b64_str = base64.b64encode(buffer).decode('utf-8')
        latest_map_b64 = f'data:image/png;base64,{b64_str}'
        map_counter += 1
        
    except Exception as e:
        pass

# Subscribe to map, odometry, and laser scan (RViz-style)
# Note: These will be re-subscribed after connection in setup_publishers()
map_listener = roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
map_listener.subscribe(map_callback)

odom_listener = None
scan_listener = None

# Timer to render combined RViz-style map
_render_timer_started = False

def start_map_render_timer():
    """Start background thread to render map with robot and laser overlay."""
    global _render_timer_started
    if _render_timer_started:
        return
    _render_timer_started = True
    
    def render_loop():
        while True:
            render_rviz_map()
            time.sleep(0.1)  # 10 FPS
    
    t = threading.Thread(target=render_loop, daemon=True)
    t.start()
    print("🗺️ Map render timer started")

def gas_callback(message):
    if gas_knob:
        val = float(message.get('data', 0))
        gas_knob.set_value(val)

gas_listener = roslibpy.Topic(client, '/gas_sensor', 'std_msgs/Float32')
gas_listener.subscribe(gas_callback)

def log_callback(message):
    if log_container:
        timestamp = time.strftime("%H:%M:%S")
        log_container.push(f"[{timestamp}] {message['data']}")

listener = roslibpy.Topic(client, '/robot_log', 'std_msgs/String')
listener.subscribe(log_callback)

def battery_callback(message):
    val = message.get('percentage', 0)
    if battery_knob:
        battery_knob.set_value(val)
    if battery_chart:
        battery_chart.options['series'][0]['data'].append(val)
        if len(battery_chart.options['series'][0]['data']) > 20:
            battery_chart.options['series'][0]['data'].pop(0)
        battery_chart.update()

battery_listener = roslibpy.Topic(client, '/battery_state', 'sensor_msgs/BatteryState')
battery_listener.subscribe(battery_callback)

def connect_to_ros_thread():
    publishers_setup = False
    while True:
        try:
            if not client.is_connected:
                client.run()
            elif not publishers_setup:
                setup_publishers()
                publishers_setup = True
            time.sleep(2)
        except:
            publishers_setup = False
            time.sleep(2)

def update_connection_status():
    global connection_notified, status_label
    if status_label:
        if client.is_connected:
            status_label.text = 'ONLINE'
            status_label.classes(remove='text-red-500 text-yellow-500', add='text-green-400')
            if not connection_notified:
                ui.notify('Connected to Robot!', type='positive')
                connection_notified = True
        else:
            status_label.text = 'OFFLINE'
            status_label.classes(remove='text-green-400 text-yellow-500', add='text-red-500')

def video_stream_loop():
    global latest_frame_b64, frame_counter
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.setsockopt_string(zmq.SUBSCRIBE, '')
    
    connected = False
    while not connected:
        try:
            socket.connect(f"tcp://{RASPBERRY_IP}:{TCP_PORT}")
            connected = True
        except Exception:
            time.sleep(1)

    while True:
        try:
            data = socket.recv()
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                results = model(frame, verbose=False)
                annotated_frame = results[0].plot()
                
                _, buffer = cv2.imencode('.jpg', annotated_frame)
                b64_string = base64.b64encode(buffer).decode('utf-8')
                latest_frame_b64 = f'data:image/jpeg;base64,{b64_string}'
                frame_counter += 1
        except:
            continue

def update_ui_content():
    global ui_frame_counter, ui_map_counter
    
    if video_image and latest_frame_b64 and (frame_counter > ui_frame_counter):
        video_image.set_source(latest_frame_b64)
        ui_frame_counter = frame_counter
    
    if map_image and latest_map_b64 and (map_counter > ui_map_counter):
        map_image.set_source(latest_map_b64)
        ui_map_counter = map_counter

def handle_keyboard(e):
    """Handle keyboard events for robot control."""
    key = e.key.lower() if hasattr(e.key, 'lower') else str(e.key).lower()
    
    if e.action.keydown:
        if key in ['w', 'arrowup']:
            move_forward()
        elif key in ['s', 'arrowdown']:
            move_backward()
        elif key in ['a', 'arrowleft']:
            turn_left()
        elif key in ['d', 'arrowright']:
            turn_right()
        elif key == ' ':
            stop_robot()
        elif key == 'escape':
            emergency_stop()
    elif e.action.keyup:
        if key in ['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright']:
            stop_robot()

@ui.page('/')
def main_page():
    global status_label, battery_chart, video_image, map_image, gas_knob, battery_knob, confidence_knob, log_container, speed_label, action_label
    
    ui.add_head_html('''
        <style>
            body { background-color: #0b0e14; overflow: hidden; }
            .nicegui-content { padding: 0; margin: 0; width: 100%; height: 100vh; }
            ::-webkit-scrollbar { display: none; }
            .glass-card {
                background: rgba(30, 41, 59, 0.7);
                border: 1px solid rgba(255, 255, 255, 0.1);
                border-radius: 16px;
                backdrop-filter: blur(10px);
                box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
            }
        </style>
    ''')
    
    # Keyboard bindings
    ui.keyboard(on_key=handle_keyboard)

    ui.dark_mode().enable()

    with ui.row().classes('w-full h-full p-4 gap-4 no-wrap items-stretch'):
        
        with ui.column().classes('w-1/4 h-full gap-4'):
            
            with ui.card().classes('glass-card w-full p-4'):
                ui.label('ROBOT STATUS').classes('text-gray-400 text-xs font-bold tracking-widest mb-2')
                with ui.row().classes('items-center justify-between w-full'):
                    with ui.row().classes('items-center gap-3'):
                        ui.icon('smart_toy', size='32px').classes('text-blue-400')
                        with ui.column().classes('gap-0'):
                            ui.label('Master Robot').classes('text-white font-bold text-lg leading-none')
                            ui.label('ID: #8821').classes('text-gray-500 text-xs')
                            ui.label(f'{RASPBERRY_IP}:{ROS_PORT}').classes('text-cyan-400 text-xs font-mono mt-1')
                    status_label = ui.label('CONNECTING').classes('text-yellow-500 font-bold text-sm')
            
            with ui.card().classes('glass-card w-full h-1/4'):
                ui.label('SIGNAL STRENGTH').classes('text-gray-400 text-xs font-bold tracking-widest mb-1')
                ui.echart({
                    'grid': {'top': 10, 'bottom': 10, 'left': 0, 'right': 0},
                    'xAxis': {'type': 'category', 'show': False},
                    'yAxis': {'type': 'value', 'show': False},
                    'series': [{
                        'data': [10, 40, 30, 70, 50, 80, 60, 90, 40, 70],
                        'type': 'line', 'smooth': True, 'showSymbol': False,
                        'lineStyle': {'color': '#60a5fa', 'width': 3},
                        'areaStyle': {'color': {'type': 'linear', 'x': 0, 'y': 0, 'x2': 0, 'y2': 1, 'colorStops': [{'offset': 0, 'color': '#60a5fa'}, {'offset': 1, 'color': 'transparent'}]}}
                    }]
                }).classes('w-full h-full')

            with ui.card().classes('glass-card w-full flex-grow flex flex-col justify-between'):
                with ui.column().classes('w-full h-full'):
                    ui.label('INCIDENT REPORT').classes('text-gray-400 text-xs font-bold tracking-widest mb-2')
                    log_container = ui.log().classes('w-full h-full text-xs text-green-400 font-mono bg-transparent')

        with ui.column().classes('w-2/4 h-full gap-4'):
            
            with ui.card().classes('glass-card w-full h-1/2 p-0 relative overflow-hidden bg-black border-2 border-blue-900'):
                ui.label('LIVE FEED').classes('absolute top-3 left-3 z-10 text-white bg-red-600 px-2 py-0.5 text-xs rounded font-bold shadow-lg')
                video_image = ui.interactive_image().classes('w-full h-full object-contain')

            with ui.row().classes('w-full h-1/4 gap-4 no-wrap'):
                with ui.card().classes('glass-card w-1/3 flex flex-col items-center justify-center py-2'):
                    ui.label('GAS LEVEL').classes('text-xs text-blue-300 font-bold mb-1')
                    gas_knob = ui.knob(0, min=0, max=1000, show_value=True, track_color='grey-9', color='cyan-4').props('readonly size=70px thickness=0.2')
                    ui.label('PPM').classes('text-xs text-gray-500')
                
                with ui.card().classes('glass-card w-1/3 flex flex-col items-center justify-center py-2'):
                    ui.label('BATTERY').classes('text-xs text-orange-300 font-bold mb-1')
                    battery_knob = ui.knob(0, min=0, max=100, show_value=True, track_color='grey-9', color='orange-4').props('readonly size=70px thickness=0.2')
                    ui.label('%').classes('text-xs text-gray-500')

                with ui.card().classes('glass-card w-1/3 flex flex-col items-center justify-center py-2'):
                    ui.label('CONFIDENCE').classes('text-xs text-purple-300 font-bold mb-1')
                    confidence_knob = ui.knob(85, min=0, max=100, show_value=True, track_color='grey-9', color='purple-4').props('readonly size=70px thickness=0.2')
                    ui.label('YOLO').classes('text-xs text-gray-500')

            with ui.card().classes('glass-card w-full flex-grow'):
                ui.label('SENSOR HISTORY').classes('text-gray-400 text-xs font-bold tracking-widest')
                battery_chart = ui.echart({
                    'grid': {'top': '20%', 'bottom': '10%', 'left': '5%', 'right': '5%'},
                    'xAxis': {'type': 'category', 'data': [str(i) for i in range(20)], 'axisLine': {'lineStyle': {'color': '#555'}}},
                    'yAxis': {'type': 'value', 'splitLine': {'lineStyle': {'color': '#333'}}, 'axisLabel': {'color': '#888'}},
                    'series': [{'type': 'line', 'data': [0]*20, 'smooth': True, 'showSymbol': False, 'color': '#f59e0b', 'areaStyle': {'opacity': 0.2}}]
                }).classes('w-full h-full')

        with ui.column().classes('w-1/4 h-full gap-4'):
            
            with ui.card().classes('glass-card w-full h-1/3 p-0 relative bg-gray-900 border-2 border-green-900'):
                ui.label('SLAM MAP').classes('absolute top-3 left-3 z-10 text-black bg-white px-2 py-0.5 text-xs rounded font-bold shadow-lg')
                map_image = ui.interactive_image().classes('w-full h-full object-contain')

            # Manual Control Panel
            with ui.card().classes('glass-card w-full p-4'):
                ui.label('MANUAL CONTROL').classes('text-gray-400 text-xs font-bold tracking-widest mb-2')
                
                # Action Status
                with ui.row().classes('w-full justify-center mb-2'):
                    action_label = ui.label('Ready').classes('text-green-400 font-bold text-lg')
                
                # D-Pad Controls
                with ui.column().classes('w-full items-center gap-1'):
                    ui.button('▲', on_click=move_forward).classes('w-14 h-12 text-xl bg-blue-600 hover:bg-blue-500 rounded-lg')
                    with ui.row().classes('gap-1'):
                        ui.button('◀', on_click=turn_left).classes('w-14 h-12 text-xl bg-blue-600 hover:bg-blue-500 rounded-lg')
                        ui.button('■', on_click=stop_robot).classes('w-14 h-12 text-xl bg-gray-600 hover:bg-gray-500 rounded-lg')
                        ui.button('▶', on_click=turn_right).classes('w-14 h-12 text-xl bg-blue-600 hover:bg-blue-500 rounded-lg')
                    ui.button('▼', on_click=move_backward).classes('w-14 h-12 text-xl bg-blue-600 hover:bg-blue-500 rounded-lg')
                
                # Speed Control
                ui.label('SPEED').classes('text-gray-500 text-xs mt-3')
                ui.slider(min=0.1, max=0.3, step=0.01, value=0.15, on_change=lambda e: update_speed(e.value)).classes('w-full')
                speed_label = ui.label('0.15 m/s').classes('text-cyan-400 text-sm text-center w-full')
                
                # Autonomous Toggle
                with ui.row().classes('w-full items-center justify-between mt-2'):
                    ui.label('Autonomous').classes('text-gray-400')
                    ui.switch(on_change=lambda e: toggle_autonomous(e.value)).classes('text-cyan-400')
            
            # Emergency Controls
            with ui.card().classes('glass-card w-full p-3'):
                ui.label('EMERGENCY').classes('text-gray-400 text-xs font-bold tracking-widest mb-2')
                with ui.row().classes('w-full gap-2'):
                    ui.button('🛑 STOP', on_click=emergency_stop).classes('flex-grow h-12 bg-red-600 hover:bg-red-500 text-white font-bold rounded-lg')
                    ui.button('✓', on_click=release_emergency).classes('w-12 h-12 bg-green-600 hover:bg-green-500 text-white font-bold rounded-lg')
            
            # Keyboard Hints
            with ui.card().classes('glass-card w-full p-3 flex-grow'):
                ui.label('KEYBOARD').classes('text-gray-400 text-xs font-bold tracking-widest mb-2')
                ui.label('W/↑ Forward').classes('text-gray-500 text-xs')
                ui.label('S/↓ Backward').classes('text-gray-500 text-xs')
                ui.label('A/← Turn Left').classes('text-gray-500 text-xs')
                ui.label('D/→ Turn Right').classes('text-gray-500 text-xs')
                ui.label('Space: Stop').classes('text-gray-500 text-xs')
                ui.label('Esc: Emergency').classes('text-gray-500 text-xs')

    ui.timer(1.0, update_connection_status)
    ui.timer(0.05, update_ui_content)

if __name__ in {"__main__", "__mp_main__"}:
    t1 = threading.Thread(target=connect_to_ros_thread, daemon=True)
    t1.start()
    t2 = threading.Thread(target=video_stream_loop, daemon=True)
    t2.start()
    
    ui.run(title='Sci-Fi Robot Dashboard', dark=True, port=8080, reload=False)