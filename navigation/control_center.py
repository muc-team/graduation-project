#!/usr/bin/env python3
"""
Complete Robot Control Dashboard
- Manual Control with keyboard/buttons
- Autonomous mode toggle
- Emergency Stop
- Live Map view
- Status monitoring
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid
from nicegui import ui, app
import threading
import numpy as np
import base64
import cv2
import time


# ============== ROS Node ==============
class DashboardNode(Node):
    def __init__(self):
        super().__init__('robot_dashboard')
        
        # Publishers
        self.manual_pub = self.create_publisher(Twist, '/manual_cmd', 10)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String, '/motor_status', self.status_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # State
        self.motor_status = 'IDLE'
        self.latest_map = None
        self.map_updated = False
        
        self.get_logger().info('Dashboard node started')
    
    def status_callback(self, msg):
        self.motor_status = msg.data
    
    def map_callback(self, msg):
        try:
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            
            img = np.full((height, width, 3), 40, dtype=np.uint8)
            img[data == 0] = [255, 255, 255]  # Free = white
            img[data == 100] = [0, 0, 200]     # Occupied = red
            img = np.flipud(img)
            
            _, buffer = cv2.imencode('.png', img)
            self.latest_map = f'data:image/png;base64,{base64.b64encode(buffer).decode()}'
            self.map_updated = True
        except:
            pass
    
    def send_manual(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.manual_pub.publish(msg)
    
    def emergency_stop(self, activate=True):
        msg = Bool()
        msg.data = activate
        self.estop_pub.publish(msg)


# ============== Global State ==============
node = None
ros_thread = None
autonomous_mode = False
current_action = 'Ready'


def start_ros():
    global node
    rclpy.init()
    node = DashboardNode()
    rclpy.spin(node)


ros_thread = threading.Thread(target=start_ros, daemon=True)
ros_thread.start()
time.sleep(2)  # Wait for ROS init


# ============== GUI ==============
ui.add_head_html('''
<style>
    body { background: #0a0a1a; }
    .control-btn {
        width: 80px !important; height: 80px !important;
        font-size: 32px !important; border-radius: 16px !important;
    }
    .control-btn:active { transform: scale(0.9); }
    .stop-btn {
        width: 80px !important; height: 80px !important;
        font-size: 18px !important; border-radius: 50% !important;
    }
    .estop-btn {
        animation: pulse 1s infinite;
    }
    @keyframes pulse {
        0%, 100% { box-shadow: 0 0 0 0 rgba(239,68,68,0.7); }
        50% { box-shadow: 0 0 0 15px rgba(239,68,68,0); }
    }
</style>
''')

# Speed settings
LINEAR = 0.2
ANGULAR = 0.5
current_cmd = 'S'


def move(cmd, action):
    global current_cmd, current_action
    if current_cmd != cmd:
        current_cmd = cmd
        current_action = action
        action_label.text = action
        
        if cmd == 'F':
            node.send_manual(LINEAR, 0)
        elif cmd == 'B':
            node.send_manual(-LINEAR, 0)
        elif cmd == 'L':
            node.send_manual(0, ANGULAR)
        elif cmd == 'R':
            node.send_manual(0, -ANGULAR)
        else:
            node.send_manual(0, 0)


def stop():
    move('S', '🛑 Stopped')
    action_label.classes(remove='text-green-400', add='text-red-400')


def emergency():
    node.emergency_stop(True)
    estop_active.set_value(True)
    action_label.text = '🚨 EMERGENCY STOP!'
    action_label.classes(remove='text-green-400', add='text-red-500')


def release_estop():
    node.emergency_stop(False)
    estop_active.set_value(False)
    action_label.text = 'Ready'


# Main layout
with ui.row().classes('w-full h-screen p-4 gap-4'):
    
    # Left Panel - Control
    with ui.column().classes('w-1/3 gap-4'):
        
        # Title
        ui.label('🤖 Robot Control Center').classes('text-2xl font-bold text-white')
        
        # Status Card
        with ui.card().classes('bg-slate-800 p-4 w-full'):
            ui.label('Status').classes('text-gray-400 text-sm')
            action_label = ui.label('Ready').classes('text-2xl font-bold text-green-400')
            with ui.row().classes('gap-2 mt-2'):
                ui.label('Mode:').classes('text-gray-400')
                mode_label = ui.label('MANUAL').classes('text-cyan-400 font-bold')
        
        # Control Pad
        with ui.card().classes('bg-slate-800 p-6'):
            ui.label('⌨️ WASD / Arrow Keys').classes('text-gray-400 text-sm mb-4 text-center w-full')
            
            with ui.column().classes('items-center'):
                # Forward
                fwd = ui.button('⬆️').classes('control-btn bg-blue-600')
                fwd.on('mousedown', lambda: move('F', '⬆️ Forward'))
                fwd.on('mouseup', stop)
                fwd.on('mouseleave', stop)
                
                # Left Stop Right
                with ui.row().classes('gap-2 my-2'):
                    left = ui.button('⬅️').classes('control-btn bg-blue-600')
                    left.on('mousedown', lambda: move('L', '⬅️ Left'))
                    left.on('mouseup', stop)
                    left.on('mouseleave', stop)
                    
                    stop_btn = ui.button('STOP').classes('stop-btn bg-gray-600 text-white')
                    stop_btn.on('click', stop)
                    
                    right = ui.button('➡️').classes('control-btn bg-blue-600')
                    right.on('mousedown', lambda: move('R', '➡️ Right'))
                    right.on('mouseup', stop)
                    right.on('mouseleave', stop)
                
                # Backward
                back = ui.button('⬇️').classes('control-btn bg-blue-600')
                back.on('mousedown', lambda: move('B', '⬇️ Backward'))
                back.on('mouseup', stop)
                back.on('mouseleave', stop)
        
        # Emergency Stop
        with ui.card().classes('bg-red-900/50 p-4 border-2 border-red-600'):
            ui.label('🚨 EMERGENCY').classes('text-red-400 font-bold text-center w-full')
            estop_active = ui.checkbox('E-Stop Active').classes('text-white')
            with ui.row().classes('gap-2 w-full justify-center mt-2'):
                ui.button('STOP ALL', on_click=emergency).classes('bg-red-600 estop-btn')
                ui.button('Release', on_click=release_estop).classes('bg-green-600')
        
        # Speed Control
        with ui.card().classes('bg-slate-800 p-4'):
            ui.label('Speed').classes('text-gray-400 text-sm')
            with ui.row().classes('items-center gap-2 w-full'):
                ui.label('Linear:').classes('text-white w-16')
                linear_slider = ui.slider(min=0.1, max=0.5, step=0.05, value=0.2).classes('flex-grow')
                linear_val = ui.label('0.2').classes('text-cyan-400 w-10')
            
            def update_linear():
                global LINEAR
                LINEAR = linear_slider.value
                linear_val.text = f'{LINEAR:.1f}'
            linear_slider.on('change', update_linear)
    
    # Right Panel - Map
    with ui.column().classes('flex-grow gap-4'):
        
        with ui.card().classes('bg-slate-800 p-4 flex-grow'):
            with ui.row().classes('justify-between items-center mb-2'):
                ui.label('🗺️ SLAM Map').classes('text-xl font-bold text-white')
                auto_switch = ui.switch('Autonomous').classes('text-white')
            
            map_image = ui.image().classes('w-full h-96 object-contain bg-gray-900 rounded')
            map_image.source = 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg=='
            
            ui.label('Waiting for map data...').classes('text-gray-500 text-center w-full mt-2').bind_visibility_from(auto_switch, 'value', value=False)
        
        # Instructions
        with ui.card().classes('bg-slate-700 p-4'):
            ui.label('📋 Quick Guide').classes('text-white font-bold mb-2')
            with ui.column().classes('text-gray-300 text-sm gap-1'):
                ui.label('• WASD or Arrow keys to move')
                ui.label('• Space = Stop')
                ui.label('• Hold button/key to move, release to stop')
                ui.label('• Emergency Stop = kills all movement')
                ui.label('• Autonomous mode = robot explores by itself')


# Keyboard handler
def handle_key(e):
    if e.action.keydown:
        if e.key in ['w', 'W', 'ArrowUp']:
            move('F', '⬆️ Forward')
        elif e.key in ['s', 'S', 'ArrowDown']:
            move('B', '⬇️ Backward')
        elif e.key in ['a', 'A', 'ArrowLeft']:
            move('L', '⬅️ Left')
        elif e.key in ['d', 'D', 'ArrowRight']:
            move('R', '➡️ Right')
        elif e.key == ' ':
            stop()
        elif e.key == 'Escape':
            emergency()
    elif e.action.keyup:
        if e.key in ['w', 'W', 's', 'S', 'a', 'A', 'd', 'D', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']:
            stop()

ui.keyboard(on_key=handle_key)


# Timer to update map
def update_map():
    if node and node.latest_map and node.map_updated:
        map_image.source = node.latest_map
        node.map_updated = False

ui.timer(0.5, update_map)


# Timer to update mode
def update_mode():
    if auto_switch.value:
        mode_label.text = 'AUTONOMOUS'
        mode_label.classes(remove='text-cyan-400', add='text-green-400')
    else:
        mode_label.text = 'MANUAL'
        mode_label.classes(remove='text-green-400', add='text-cyan-400')

ui.timer(0.5, update_mode)


ui.run(title='Robot Control Center', port=8888, dark=True, reload=False, show=False)
