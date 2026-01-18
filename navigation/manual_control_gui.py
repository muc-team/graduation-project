#!/usr/bin/env python3
"""
Robot Manual Control GUI
Simple web interface to control the robot manually for testing
Run on Raspberry Pi: python3 manual_control_gui.py
Open browser: http://localhost:8888
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nicegui import ui
import threading


class RobotController(Node):
    def __init__(self):
        super().__init__('manual_control_gui')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.get_logger().info('Manual Control GUI Node started')
    
    def send_cmd(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)
        self.current_linear = linear
        self.current_angular = angular
    
    def stop(self):
        self.send_cmd(0.0, 0.0)


# Global controller
controller = None
LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.5


def ros_spin_thread():
    global controller
    rclpy.init()
    controller = RobotController()
    rclpy.spin(controller)


# Start ROS in background
ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
ros_thread.start()

# Wait for controller to initialize
import time
time.sleep(1)


# ============== GUI ==============

ui.add_head_html('''
<style>
    body { 
        background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%); 
        min-height: 100vh;
    }
    .control-btn {
        width: 100px !important;
        height: 100px !important;
        font-size: 40px !important;
        border-radius: 20px !important;
        transition: all 0.2s !important;
    }
    .control-btn:hover {
        transform: scale(1.1);
    }
    .control-btn:active {
        transform: scale(0.95);
    }
    .stop-btn {
        width: 100px !important;
        height: 100px !important;
        font-size: 24px !important;
        border-radius: 50% !important;
        animation: pulse 2s infinite;
    }
    @keyframes pulse {
        0%, 100% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.7); }
        50% { box-shadow: 0 0 0 20px rgba(239, 68, 68, 0); }
    }
</style>
''')

with ui.column().classes('w-full items-center justify-center p-8'):
    
    # Title
    ui.label('🤖 Robot Manual Control').classes('text-4xl font-bold text-white mb-2')
    ui.label('Test Mode - LiDAR Not Required').classes('text-gray-400 mb-8')
    
    # Status Card
    with ui.card().classes('bg-slate-800 p-4 mb-8 w-96'):
        with ui.row().classes('items-center gap-4'):
            status_icon = ui.icon('radio_button_checked', size='24px').classes('text-green-400')
            status_label = ui.label('Ready').classes('text-green-400 font-bold text-xl')
    
    # Control Pad
    with ui.card().classes('bg-slate-800/50 p-8 rounded-3xl'):
        
        # Forward
        with ui.row().classes('justify-center mb-4'):
            ui.button('⬆️', on_click=lambda: (controller.send_cmd(LINEAR_SPEED, 0), update_status('Forward'))).classes('control-btn bg-blue-600')
        
        # Left, Stop, Right
        with ui.row().classes('justify-center gap-4 mb-4'):
            ui.button('⬅️', on_click=lambda: (controller.send_cmd(0, ANGULAR_SPEED), update_status('Left'))).classes('control-btn bg-blue-600')
            ui.button('STOP', on_click=lambda: (controller.stop(), update_status('Stopped', 'red'))).classes('stop-btn bg-red-600 text-white font-bold')
            ui.button('➡️', on_click=lambda: (controller.send_cmd(0, -ANGULAR_SPEED), update_status('Right'))).classes('control-btn bg-blue-600')
        
        # Backward
        with ui.row().classes('justify-center'):
            ui.button('⬇️', on_click=lambda: (controller.send_cmd(-LINEAR_SPEED, 0), update_status('Backward'))).classes('control-btn bg-blue-600')
    
    # Speed Controls
    with ui.card().classes('bg-slate-800/50 p-6 mt-8 w-96'):
        ui.label('Speed Settings').classes('text-white font-bold mb-4')
        
        with ui.row().classes('items-center gap-4 w-full'):
            ui.label('Linear:').classes('text-gray-300 w-20')
            linear_slider = ui.slider(min=0.1, max=0.5, step=0.05, value=0.2).classes('flex-grow')
            linear_label = ui.label('0.2').classes('text-cyan-400 w-12')
        
        with ui.row().classes('items-center gap-4 w-full'):
            ui.label('Angular:').classes('text-gray-300 w-20')
            angular_slider = ui.slider(min=0.1, max=1.0, step=0.1, value=0.5).classes('flex-grow')
            angular_label = ui.label('0.5').classes('text-cyan-400 w-12')
        
        def update_linear():
            global LINEAR_SPEED
            LINEAR_SPEED = linear_slider.value
            linear_label.text = f'{LINEAR_SPEED:.1f}'
        
        def update_angular():
            global ANGULAR_SPEED
            ANGULAR_SPEED = angular_slider.value
            angular_label.text = f'{ANGULAR_SPEED:.1f}'
        
        linear_slider.on('change', update_linear)
        angular_slider.on('change', update_angular)
    
    # Keyboard hint
    ui.label('💡 Use buttons above to control the robot').classes('text-gray-500 mt-8')
    ui.label('⚠️ Always have someone ready to press STOP!').classes('text-yellow-400 mt-2')


def update_status(text, color='green'):
    status_label.text = text
    if color == 'red':
        status_label.classes(remove='text-green-400 text-blue-400', add='text-red-400')
        status_icon.classes(remove='text-green-400 text-blue-400', add='text-red-400')
    else:
        status_label.classes(remove='text-red-400', add='text-blue-400')
        status_icon.classes(remove='text-red-400', add='text-blue-400')


ui.run(title='Robot Control', port=8888, dark=True, reload=False)
