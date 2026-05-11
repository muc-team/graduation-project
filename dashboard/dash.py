import cv2
import os
import glob
import zmq
import time
import torch
import base64
import requests
import roslibpy
import threading
import numpy as np
import torch.nn as nn
from nicegui import ui, app
from ultralytics import YOLO


class ConcatHead(nn.Module):
    """Concatenation layer for Detect heads (Custom added for dual-head YOLO)."""
    def __init__(self, nc1=80, nc2=1, ch=()):
        super().__init__()
        self.nc1 = nc1
        self.nc2 = nc2

    def forward(self, x):
        if isinstance(x[0], tuple):
            preds1 = x[0][0]
            preds2 = x[1][0]
        elif isinstance(x[0], list):
            return [torch.cat((x0, x1), dim=1) for x0, x1 in zip(x[0], x[1])]
        else:
            preds1 = x[0]
            preds2 = x[1]

        preds = torch.cat((preds1[:, :4, :], preds2[:, :4, :]), dim=2)

        shape = list(preds1.shape)
        shape[-1] *= 2
        preds1_extended = torch.zeros(shape, device=preds1.device, dtype=preds1.dtype)
        preds1_extended[..., : preds1.shape[-1]] = preds1

        shape = list(preds2.shape)
        shape[-1] *= 2
        preds2_extended = torch.zeros(shape, device=preds2.device, dtype=preds2.dtype)
        preds2_extended[..., preds2.shape[-1] :] = preds2

        preds = torch.cat((preds, preds1_extended[:, 4:, :]), dim=1)
        preds = torch.cat((preds, preds2_extended[:, 4:, :]), dim=1)

        if isinstance(x[0], tuple):
            return (preds, x[0][1])
        else:
            return preds

# Monkey-patch ConcatHead into ultralytics so torch.load() can find it
# when deserializing model checkpoints that were trained with this custom head.
import ultralytics.nn.modules.conv as _conv_module
_conv_module.ConcatHead = ConcatHead

AVAILABLE_ROBOTS = ['robot.local', 'robot2.local', 'robot3.local']
ROBOT_PROFILES = {
    'robot.local':  {'name': 'Alpha', 'icon': 'smart_toy', 'color': 'text-blue-400'},
    'robot2.local': {'name': 'Beta', 'icon': 'precision_manufacturing', 'color': 'text-orange-400'},
    'robot3.local': {'name': 'Gamma', 'icon': 'memory', 'color': 'text-green-400', 'esp32': True},
}
RASPBERRY_IP = AVAILABLE_ROBOTS[0]
ROS_PORT, TCP_PORT = 9090, 5555

# --- ESP32 (Robot 3) state ---
def _is_esp32(ip=None):
    """Check if the given (or current) robot is an ESP32 HTTP robot."""
    return ROBOT_PROFILES.get(ip or RASPBERRY_IP, {}).get('esp32', False)

esp32_telemetry = {'d': 0, 'g': 0, 'x': 0, 'y': 0, 'a': 0}
_esp32_connected = threading.Event()
_esp32_session = requests.Session()  # persistent HTTP keep-alive for low-latency commands

# Single-slot command queue: always keeps only the latest command
import queue as _queue_mod
_esp32_cmd_queue = _queue_mod.Queue(maxsize=1)

def _esp32_worker():
    """Dedicated thread that sends ESP32 HTTP commands from the queue."""
    while True:
        direction = _esp32_cmd_queue.get()  # blocks until a command arrives
        try:
            _esp32_session.get(
                f'http://{RASPBERRY_IP}/control?dir={direction}',
                timeout=0.5,
            )
        except Exception:
            pass

threading.Thread(target=_esp32_worker, daemon=True).start()

_zmq_reconnect_flag = threading.Event()  # signals the video thread to reconnect

# --- Model Management ---
MODELS_DIR = os.path.join(os.path.dirname(__file__), '..', 'models')

def get_available_models():
    """Scan the models directory and return a list of .pt filenames."""
    pattern = os.path.join(MODELS_DIR, '*.pt')
    return sorted([os.path.basename(p) for p in glob.glob(pattern)])

available_models = get_available_models()
current_model_name = 'yolov8n-fire.pt' if 'yolov8n-fire.pt' in available_models else (available_models[0] if available_models else 'yolov8n-fire.pt')
model = YOLO(os.path.join(MODELS_DIR, current_model_name))
model_lock = threading.Lock()  # protects model reloads
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
model_select = None
model_label = None
action_label = None
robot_select = None
robot_ip_label = None
robot_name_label = None
robot_icon = None
robot_cards = {}       # {ip: card element} – for active-border highlighting
robot_status_icons = {}  # {ip: icon element} – per-robot online/offline dot

encoder_labels = [None, None, None, None]
encoder_listener = None

# Per-robot network reachability (updated by background pinger)
robot_reachable = {ip: False for ip in AVAILABLE_ROBOTS}

latest_frame_b64 = None 
latest_map_b64 = None 

frame_counter = 0        
ui_frame_counter = 0     
map_counter = 0          
ui_map_counter = 0

# Manual Control State
current_speed = 0.15
held_key = None
autonomous_mode = False

# ROS Publishers (initialize after connection)
manual_topic = None
_publishers_setup = False
explore_topic = None
goal_topic = None
nav_status_listener = None

# Navigation goal state
nav_goal = {'x': None, 'y': None}
nav_status = 'IDLE'
nav_status_label = None

def setup_publishers():
    """Setup ROS publishers and additional subscribers after connection."""
    global manual_topic, explore_topic, odom_listener, scan_listener
    global goal_topic, nav_status_listener
    if client.is_connected:
        manual_topic = roslibpy.Topic(client, '/manual_cmd', 'geometry_msgs/Twist')
        manual_topic.advertise()
        
        explore_topic = roslibpy.Topic(client, '/explore_enable', 'std_msgs/Bool')
        explore_topic.advertise()
        
        # Goal pose publisher (for click-to-navigate)
        goal_topic = roslibpy.Topic(client, '/goal_pose', 'geometry_msgs/PoseStamped')
        goal_topic.advertise()
        
        # Subscribe to odom and scan for RViz-style map
        odom_listener = roslibpy.Topic(client, '/odom', 'nav_msgs/Odometry')
        odom_listener.subscribe(pose_callback)
        
        scan_listener = roslibpy.Topic(client, '/scan', 'sensor_msgs/LaserScan')
        scan_listener.subscribe(scan_callback)
        
        # Subscribe to navigation status from robot2_goto
        nav_status_listener = roslibpy.Topic(client, '/nav_status', 'std_msgs/String')
        nav_status_listener.subscribe(nav_status_callback)
        
        # Start render timer
        start_map_render_timer()
        
        print("✅ ROS Publishers and subscribers ready")

def _esp32_cmd(direction: str):
    """Queue an HTTP control command — drops any stale pending command first."""
    # Drain any old command so only the newest one is sent
    try:
        _esp32_cmd_queue.get_nowait()
    except _queue_mod.Empty:
        pass
    try:
        _esp32_cmd_queue.put_nowait(direction)
    except _queue_mod.Full:
        pass

def send_twist(linear: float, angular: float):
    """Send velocity command to robot."""
    global manual_topic
    if _is_esp32():
        # Map twist to ESP32 direction letters
        if linear > 0:
            _esp32_cmd('F')
        elif linear < 0:
            _esp32_cmd('B')
        elif angular > 0:
            _esp32_cmd('L')
        elif angular < 0:
            _esp32_cmd('R')
        else:
            _esp32_cmd('S')
        return
    if manual_topic and client.is_connected:
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
    global held_key, nav_goal
    held_key = None
    nav_goal = {'x': None, 'y': None}
    send_twist(0, 0)
    update_action("Stopped")

def toggle_autonomous(enabled: bool):
    global autonomous_mode
    autonomous_mode = enabled
    if _is_esp32():
        ui.notify('Autonomous mode not available on ESP32 robot', type='warning')
        return
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

def swap_model(new_model_name: str):
    """Swap the YOLO model at runtime."""
    global model, current_model_name
    if new_model_name == current_model_name:
        return
    model_path = os.path.join(MODELS_DIR, new_model_name)
    if not os.path.isfile(model_path):
        ui.notify(f'Model file not found: {new_model_name}', type='negative')
        return
    try:
        ui.notify(f'Loading model: {new_model_name}…', type='info')
        new_model = YOLO(model_path)
        with model_lock:
            model = new_model
            current_model_name = new_model_name
        if model_label:
            model_label.text = current_model_name
        ui.notify(f'Model switched to {new_model_name}', type='positive')
    except Exception as e:
        ui.notify(f'Failed to load model: {e}', type='negative')

def swap_robot(new_ip: str):
    """Switch to a different robot by reconnecting ROS/ZMQ or ESP32 HTTP."""
    global RASPBERRY_IP, client, connection_notified, _publishers_setup
    global manual_topic, explore_topic, goal_topic, nav_status_listener
    global map_listener, gas_listener, listener, battery_listener, encoders_listener
    global odom_listener, scan_listener, status_listener
    global latest_frame_b64, latest_map_b64
    global nav_goal, nav_status

    if new_ip == RASPBERRY_IP:
        return

    ui.notify(f'Switching to {new_ip}…', type='info')

    # --- Tear down old ROS connection (safe even if already closed) ---
    try:
        client.close()
    except Exception:
        pass

    # Reset shared state
    manual_topic = None
    explore_topic = None
    goal_topic = None
    nav_status_listener = None
    odom_listener = None
    scan_listener = None
    status_listener = None
    connection_notified = False
    _publishers_setup = False
    latest_frame_b64 = None
    latest_map_b64 = None
    nav_goal = {'x': None, 'y': None}
    nav_status = 'IDLE'
    _esp32_connected.clear()

    # Update IP
    RASPBERRY_IP = new_ip

    if _is_esp32(new_ip):
        # --- ESP32 robot: no ROS, no ZMQ ---
        _zmq_reconnect_flag.set()   # tell ZMQ thread to disconnect
        _esp32_connected.set()       # wake up ESP32 telemetry thread
    else:
        # --- ROS-based robot: reconnect ROS + ZMQ ---
        client = roslibpy.Ros(host=RASPBERRY_IP, port=ROS_PORT)

        map_listener = roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
        map_listener.subscribe(map_callback)

        gas_listener = roslibpy.Topic(client, '/gas_sensor', 'std_msgs/Float32')
        gas_listener.subscribe(gas_callback)

        listener = roslibpy.Topic(client, '/robot_log', 'std_msgs/String')
        listener.subscribe(log_callback)

        status_listener = roslibpy.Topic(client, '/motor_status', 'std_msgs/String')
        status_listener.subscribe(status_callback)

        encoders_listener = roslibpy.Topic(client, '/encoders', 'std_msgs/Int32MultiArray')
        encoders_listener.subscribe(encoders_callback)

        # Reset encoder display for new robot
        latest_encoders[0] = '0'
        latest_encoders[1] = '0'
        latest_encoders[2] = '0'
        latest_encoders[3] = '0'

        # Signal video thread to reconnect ZMQ
        _zmq_reconnect_flag.set()

    # --- Update UI labels (common) ---
    if robot_ip_label:
        robot_ip_label.text = f'{RASPBERRY_IP}' if _is_esp32(new_ip) else f'{RASPBERRY_IP}:{ROS_PORT}'
    if status_label:
        status_label.text = 'CONNECTING'
        status_label.classes(remove='text-green-400 text-red-500', add='text-yellow-500')
    # Update robot name & icon
    profile = ROBOT_PROFILES.get(new_ip, {'name': new_ip, 'icon': 'smart_toy', 'color': 'text-blue-400'})
    if robot_name_label:
        robot_name_label.text = profile['name']
    if robot_icon:
        robot_icon._props['name'] = profile['icon']
        robot_icon.classes(remove='text-blue-400 text-orange-400 text-green-400', add=profile['color'])
        robot_icon.update()
    # Clear stale frames from previous robot
    if video_image:
        video_image.set_source('')
    if map_image:
        map_image.set_source('')

    ui.notify(f'Now targeting {profile["name"]} ({new_ip})', type='positive')

# Map data storage for RViz-style rendering
map_info = {'width': 0, 'height': 0, 'resolution': 0.05, 'origin_x': 0, 'origin_y': 0, 'data': None}
robot_pose = {'x': 0, 'y': 0, 'theta': 0}
laser_points = []

def nav_status_callback(msg):
    """Receive navigation status from robot2_goto node."""
    global nav_status
    nav_status = msg.get('data', 'IDLE')

def handle_map_click(e):
    """Handle click on SLAM map to send navigation goal."""
    global nav_goal
    if e.type != 'mousedown':
        return
    # Get click coordinates within the image
    click_x = e.image_x
    click_y = e.image_y
    
    w = map_info['width']
    h = map_info['height']
    res = map_info['resolution']
    origin_x = map_info['origin_x']
    origin_y = map_info['origin_y']
    
    if w == 0 or h == 0 or res == 0:
        return
    
    # Compute the scale factor used in render_rviz_map
    scale = max(2, min(4, 600 // max(w, h)))
    
    # Convert pixel coordinates to world coordinates
    world_x = origin_x + (click_x / scale) * res
    world_y = origin_y + (h - click_y / scale) * res
    
    nav_goal['x'] = world_x
    nav_goal['y'] = world_y
    
    # Publish goal to ROS
    if goal_topic and client.is_connected:
        goal_topic.publish(roslibpy.Message({
            'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': 'odom'},
            'pose': {
                'position': {'x': world_x, 'y': world_y, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            }
        }))
        ui.notify(f'🎯 Goal sent: ({world_x:.2f}, {world_y:.2f})', type='positive')
    else:
        ui.notify('Not connected to robot', type='warning')

def map_callback(msg):
    """Store map data for rendering."""
    global map_info
    try:
        map_info['width'] = msg['info']['width']
        map_info['height'] = msg['info']['height']
        map_info['resolution'] = msg['info']['resolution']
        map_info['origin_x'] = msg['info']['origin']['position']['x']
        map_info['origin_y'] = msg['info']['origin']['position']['y']
        
        # rosbridge_server might base64 encode byte arrays
        if isinstance(msg['data'], str):
            import base64
            decoded = base64.b64decode(msg['data'])
            map_info['data'] = np.frombuffer(decoded, dtype=np.int8)
        else:
            map_info['data'] = np.array(msg['data'], dtype=np.int8)
    except Exception as e:
        print(f"Error in map_callback: {e}")

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
        print(f"Error in pose_callback: {e}")

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
        
        # Flip image vertically so +Y is up (matching robot's ry projection)
        img = np.flipud(img)
        
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
        
        # Draw robot (green circle with direction arrow - like RViz)
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
        
        # Draw navigation goal marker (red crosshair)
        if nav_goal['x'] is not None and nav_goal['y'] is not None:
            gx = int((nav_goal['x'] - origin_x) / res * scale)
            gy = int((h - (nav_goal['y'] - origin_y) / res) * scale)
            if 0 <= gx < w * scale and 0 <= gy < h * scale:
                cross_size = max(8, robot_size)
                # Red crosshair
                cv2.line(img, (gx - cross_size, gy), (gx + cross_size, gy), (0, 0, 255), 2)
                cv2.line(img, (gx, gy - cross_size), (gx, gy + cross_size), (0, 0, 255), 2)
                # Red circle
                cv2.circle(img, (gx, gy), cross_size // 2, (0, 0, 255), 2)
                
                # Draw line from robot to goal (yellow dashed-like)
                if 0 <= rx < w * scale and 0 <= ry < h * scale:
                    cv2.line(img, (rx, ry), (gx, gy), (0, 200, 255), 1, cv2.LINE_AA)
        
        # Draw nav status text on map
        if nav_status and nav_status != 'IDLE':
            status_text = nav_status.split(':')[0]
            cv2.putText(img, status_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 200, 255), 2, cv2.LINE_AA)
        
        # Encode to base64
        _, buffer = cv2.imencode('.png', img)
        b64_str = base64.b64encode(buffer).decode('utf-8')
        latest_map_b64 = f'data:image/png;base64,{b64_str}'
        map_counter += 1
        
    except Exception as e:
        print(f"Error in render_rviz_map: {e}")

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
            try:
                render_rviz_map()
            except Exception as e:
                print(f"Map Render Thread Error: {e}")
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

# Store latest encoder values
latest_encoders = ['0', '0', '0', '0']

def status_callback(message):
    global latest_encoders
    data = message.get('data', '')
    if data.startswith('STS:'):
        parts = data[4:].split(',')
        if len(parts) >= 6:
            latest_encoders[0] = parts[2]
            latest_encoders[1] = parts[3]
            latest_encoders[2] = parts[4]
            latest_encoders[3] = parts[5]

def encoders_callback(message):
    """Fallback: read /encoders (Int32MultiArray) from robot2_bridge."""
    global latest_encoders
    data = message.get('data', [])
    if len(data) >= 4:
        latest_encoders[0] = str(data[0])
        latest_encoders[1] = str(data[1])
        latest_encoders[2] = str(data[2])
        latest_encoders[3] = str(data[3])

status_listener = roslibpy.Topic(client, '/motor_status', 'std_msgs/String')
status_listener.subscribe(status_callback)

encoders_listener = roslibpy.Topic(client, '/encoders', 'std_msgs/Int32MultiArray')
encoders_listener.subscribe(encoders_callback)

def connect_to_ros_thread():
    global _publishers_setup
    while True:
        try:
            # Skip ROS connection attempts when an ESP32 robot is active
            if _is_esp32():
                time.sleep(2)
                continue
            if not client.is_connected:
                _publishers_setup = False
                client.run()
            elif not _publishers_setup:
                setup_publishers()
                _publishers_setup = True
            time.sleep(2)
        except:
            _publishers_setup = False
            time.sleep(2)

def esp32_telemetry_loop():
    """Background thread: polls ESP32 /telemetry and updates dashboard gauges."""
    global esp32_telemetry
    while True:
        if not _is_esp32():
            time.sleep(1)
            continue
        try:
            r = _esp32_session.get(f'http://{RASPBERRY_IP}/telemetry', timeout=1)
            data = r.json()
            esp32_telemetry = data
            _esp32_connected.set()

            # Feed ESP32 gas value into the gas knob (same as gas_callback)
            if gas_knob:
                gas_knob.set_value(float(data.get('g', 0)))

        except Exception:
            _esp32_connected.clear()
        time.sleep(0.5)

def _ping_robot(ip):
    """Check if a robot is reachable on the network."""
    profile = ROBOT_PROFILES.get(ip, {})
    try:
        if profile.get('esp32'):
            r = requests.get(f'http://{ip}/telemetry', timeout=1.5)
            return r.status_code == 200
        else:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.5)
            s.connect((ip, ROS_PORT))
            s.close()
            return True
    except Exception:
        return False

def robot_ping_loop():
    """Background thread: pings all robots every 3s to track reachability."""
    while True:
        for ip in AVAILABLE_ROBOTS:
            robot_reachable[ip] = _ping_robot(ip)
        time.sleep(3)

def update_connection_status():
    global connection_notified, status_label
    # --- Update the active-robot status label ---
    if status_label:
        if _is_esp32():
            if _esp32_connected.is_set():
                status_label.text = 'ONLINE'
                status_label.classes(remove='text-red-500 text-yellow-500', add='text-green-400')
                if not connection_notified:
                    ui.notify('Connected to ESP32 Robot!', type='positive')
                    connection_notified = True
            else:
                status_label.text = 'OFFLINE'
                status_label.classes(remove='text-green-400 text-yellow-500', add='text-red-500')
        elif client.is_connected:
            status_label.text = 'ONLINE'
            status_label.classes(remove='text-red-500 text-yellow-500', add='text-green-400')
            if not connection_notified:
                ui.notify('Connected to Robot!', type='positive')
                connection_notified = True
        else:
            status_label.text = 'OFFLINE'
            status_label.classes(remove='text-green-400 text-yellow-500', add='text-red-500')

    # --- Update per-robot fleet cards ---
    for ip in AVAILABLE_ROBOTS:
        reachable = robot_reachable.get(ip, False)
        icon_el = robot_status_icons.get(ip)
        card_el = robot_cards.get(ip)
        if icon_el:
            if reachable:
                icon_el._props['name'] = 'wifi'
                icon_el.classes(remove='text-red-500', add='text-green-400')
            else:
                icon_el._props['name'] = 'wifi_off'
                icon_el.classes(remove='text-green-400', add='text-red-500')
            icon_el.update()
        if card_el:
            if ip == RASPBERRY_IP:
                card_el.classes(remove='border-transparent', add='border-cyan-400')
            else:
                card_el.classes(remove='border-cyan-400', add='border-transparent')

def video_stream_loop():
    global latest_frame_b64, frame_counter

    context = None
    socket = None
    current_endpoint = None

    def _cleanup():
        """Forcefully close socket and context."""
        nonlocal socket, context, current_endpoint
        try:
            if socket is not None:
                if current_endpoint:
                    try:
                        socket.disconnect(current_endpoint)
                    except Exception:
                        pass
                socket.setsockopt(zmq.LINGER, 0)
                socket.close()
        except Exception:
            pass
        try:
            if context is not None:
                context.term()
        except Exception:
            pass
        socket = None
        context = None
        current_endpoint = None
        time.sleep(0.3)  # let OS release the resources

    while True:
        # --- Clean up any previous socket ---
        _cleanup()
        _zmq_reconnect_flag.clear()

        # If the current robot is ESP32, no ZMQ stream — just idle
        if _is_esp32():
            _zmq_reconnect_flag.wait()   # block until robot is switched
            continue

        # --- Create fresh socket ---
        target_ip = RASPBERRY_IP  # snapshot current target
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.CONFLATE, 1)
        socket.setsockopt_string(zmq.SUBSCRIBE, '')
        socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1s receive timeout
        socket.setsockopt(zmq.LINGER, 0)        # don't block on close

        current_endpoint = f"tcp://{target_ip}:{TCP_PORT}"
        try:
            socket.connect(current_endpoint)
        except Exception:
            time.sleep(1)
            continue  # retry with fresh socket

        print(f"📷 ZMQ connected to {current_endpoint}")

        # --- Receive loop ---
        while True:
            if _zmq_reconnect_flag.is_set():
                print(f"📷 ZMQ reconnect requested, leaving {current_endpoint}")
                break

            try:
                data = socket.recv()
                nparr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if frame is not None:
                    with model_lock:
                        current = model
                    results = current(frame, verbose=False)
                    annotated_frame = results[0].plot()

                    _, buffer = cv2.imencode('.jpg', annotated_frame)
                    b64_string = base64.b64encode(buffer).decode('utf-8')
                    latest_frame_b64 = f'data:image/jpeg;base64,{b64_string}'
                    frame_counter += 1
            except zmq.Again:
                continue  # timeout — loop to check reconnect flag
            except Exception:
                continue

def update_ui_content():
    global ui_frame_counter, ui_map_counter
    
    if video_image and latest_frame_b64 and (frame_counter > ui_frame_counter):
        video_image.set_source(latest_frame_b64)
        ui_frame_counter = frame_counter
    
    if map_image and latest_map_b64 and (map_counter > ui_map_counter):
        map_image.set_source(latest_map_b64)
        ui_map_counter = map_counter
        
    # Safely update encoder labels
    for i in range(4):
        if encoder_labels[i] and encoder_labels[i].text != latest_encoders[i]:
            encoder_labels[i].text = latest_encoders[i]

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
    elif e.action.keyup:
        if key in ['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright']:
            stop_robot()

@ui.page('/')
def main_page():
    global status_label, battery_chart, video_image, map_image, gas_knob, battery_knob, confidence_knob, log_container, speed_label, action_label, model_select, model_label, robot_select, robot_ip_label, robot_name_label, robot_icon
    
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
            
            # --- SWARM FLEET: clickable robot list ---
            with ui.card().classes('glass-card w-full p-4'):
                ui.label('SWARM FLEET').classes('text-gray-400 text-xs font-bold tracking-widest mb-3')
                status_label = ui.label('CONNECTING').classes('text-yellow-500 font-bold text-sm hidden')  # hidden but still updated
                for _ip in AVAILABLE_ROBOTS:
                    _prof = ROBOT_PROFILES.get(_ip, {'name': _ip, 'icon': 'smart_toy', 'color': 'text-blue-400'})
                    _is_active = (_ip == RASPBERRY_IP)
                    _border = 'border-cyan-400' if _is_active else 'border-transparent'
                    _card = ui.card().classes(
                        f'w-full p-3 cursor-pointer border-2 {_border} '
                        'rounded-xl transition-all duration-200 hover:bg-white/5'
                    ).style('background: rgba(15, 23, 42, 0.6);')
                    _card.on('click', lambda _ip=_ip: swap_robot(_ip))
                    robot_cards[_ip] = _card
                    with _card:
                        with ui.row().classes('items-center justify-between w-full'):
                            with ui.row().classes('items-center gap-3'):
                                ui.icon(_prof['icon'], size='26px').classes(_prof['color'])
                                with ui.column().classes('gap-0'):
                                    ui.label(_prof['name']).classes('text-white font-bold text-sm leading-none')
                                    ui.label(_ip).classes('text-gray-500 text-xs font-mono mt-0.5')
                            _st_icon = ui.icon('wifi_off', size='20px').classes('text-red-500')
                            robot_status_icons[_ip] = _st_icon

            with ui.card().classes('glass-card w-full flex-grow flex flex-col justify-between'):
                with ui.column().classes('w-full h-full'):
                    ui.label('INCIDENT REPORT').classes('text-gray-400 text-xs font-bold tracking-widest mb-2')
                    log_container = ui.log().classes('w-full h-full text-xs text-green-400 font-mono bg-transparent')

        with ui.column().classes('w-2/4 h-full gap-4'):
            
            with ui.card().classes('glass-card w-full h-1/2 p-0 relative overflow-hidden bg-black border-2 border-blue-900'):
                ui.label('LIVE FEED').classes('absolute top-3 left-3 z-10 text-white bg-red-600 px-2 py-0.5 text-xs rounded font-bold shadow-lg')
                # Model selector overlay (top-right of live feed)
                with ui.row().classes('absolute top-2 right-3 z-10 items-center gap-2'):
                    ui.icon('model_training', size='20px').classes('text-cyan-400')
                    model_select = ui.select(
                        options=available_models,
                        value=current_model_name,
                        on_change=lambda e: swap_model(e.value),
                    ).props('dense borderless dark color=cyan-4').classes('text-white min-w-[160px]').style('font-size: 12px;')
                video_image = ui.interactive_image().classes('w-full h-full object-contain')

            with ui.row().classes('w-full h-1/4 gap-4 no-wrap'):
                with ui.card().classes('glass-card w-1/3 flex flex-col items-center justify-center py-2'):
                    ui.label('GAS LEVEL').classes('text-xs text-blue-300 font-bold mb-1')
                    gas_knob = ui.knob(0, min=0, max=4000, show_value=True, track_color='grey-9', color='cyan-4').props('readonly size=70px thickness=0.2')
                    ui.label('PPM').classes('text-xs text-gray-500')
                
                with ui.card().classes('glass-card w-2/3 p-3'):
                    ui.label('WHEEL ENCODERS').classes('text-gray-400 text-xs font-bold tracking-widest mb-2')
                    with ui.row().classes('w-full justify-between gap-1'):
                        with ui.column().classes('items-center gap-0 w-[48%] bg-black/30 p-2 rounded'):
                            ui.label('Front L').classes('text-[10px] text-gray-500')
                            encoder_labels[0] = ui.label('0').classes('text-sm text-cyan-400 font-mono')
                        with ui.column().classes('items-center gap-0 w-[48%] bg-black/30 p-2 rounded'):
                            ui.label('Front R').classes('text-[10px] text-gray-500')
                            encoder_labels[2] = ui.label('0').classes('text-sm text-cyan-400 font-mono')
                        with ui.column().classes('items-center gap-0 w-[48%] bg-black/30 p-2 rounded'):
                            ui.label('Rear L').classes('text-[10px] text-gray-500')
                            encoder_labels[1] = ui.label('0').classes('text-sm text-cyan-400 font-mono')
                        with ui.column().classes('items-center gap-0 w-[48%] bg-black/30 p-2 rounded'):
                            ui.label('Rear R').classes('text-[10px] text-gray-500')
                            encoder_labels[3] = ui.label('0').classes('text-sm text-cyan-400 font-mono')

        with ui.column().classes('w-1/4 h-full gap-4'):
            
            with ui.card().classes('glass-card w-full h-1/3 p-0 relative bg-gray-900 border-2 border-green-900'):
                ui.label('SLAM MAP').classes('absolute top-3 left-3 z-10 text-black bg-white px-2 py-0.5 text-xs rounded font-bold shadow-lg')
                # Click on map to set navigation goal
                map_image = ui.interactive_image(
                    on_mouse=handle_map_click,
                    events=['mousedown'],
                    cross=True
                ).classes('w-full h-full object-contain')

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
            
            

    ui.timer(1.0, update_connection_status)
    ui.timer(0.05, update_ui_content)

if __name__ in {"__main__", "__mp_main__"}:
    t1 = threading.Thread(target=connect_to_ros_thread, daemon=True)
    t1.start()
    t2 = threading.Thread(target=video_stream_loop, daemon=True)
    t2.start()
    t3 = threading.Thread(target=esp32_telemetry_loop, daemon=True)
    t3.start()
    t4 = threading.Thread(target=robot_ping_loop, daemon=True)
    t4.start()
    
    ui.run(title='Sci-Fi Robot Dashboard', dark=True, port=8080, reload=False)