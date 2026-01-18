#!/usr/bin/env python3
"""
Robot Manual Control GUI - Simple Version (No ROS)
Directly controls Arduino via Serial
Run: python3 manual_control_gui.py
Open: http://localhost:8888
"""

import serial
import time
from nicegui import ui

# ============== Serial Setup ==============
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

arduino = None
connected = False

def connect_arduino():
    global arduino, connected
    ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1', '/dev/ttyAMA0']
    for port in ports:
        try:
            arduino = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2)
            connected = True
            print(f"✅ Connected to Arduino on {port}")
            return True
        except:
            continue
    print("❌ Could not connect to Arduino")
    return False

def send_command(cmd):
    global arduino, connected
    if arduino and arduino.is_open:
        try:
            arduino.write(cmd.encode())
            print(f"Sent: {cmd}")
            return True
        except:
            return False
    return False

# Try to connect
connect_arduino()

# ============== Speed Settings ==============
LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.5

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
        cursor: pointer !important;
    }
    .control-btn:hover { transform: scale(1.1); }
    .control-btn:active { transform: scale(0.95); }
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
    ui.label('Direct Serial Control - No ROS Required').classes('text-gray-400 mb-8')
    
    # Status Card
    with ui.card().classes('bg-slate-800 p-4 mb-8 w-96'):
        with ui.row().classes('items-center gap-4'):
            if connected:
                ui.icon('radio_button_checked', size='24px').classes('text-green-400')
                status_label = ui.label('Arduino Connected ✅').classes('text-green-400 font-bold text-xl')
            else:
                ui.icon('radio_button_unchecked', size='24px').classes('text-red-400')
                status_label = ui.label('Arduino Not Connected ❌').classes('text-red-400 font-bold text-xl')
    
    # Current Action
    action_label = ui.label('Ready').classes('text-2xl text-cyan-400 font-bold mb-4')
    
    # Control Pad
    with ui.card().classes('bg-slate-800/50 p-8 rounded-3xl'):
        
        # Forward
        with ui.row().classes('justify-center mb-4'):
            def go_forward():
                send_command('F')
                action_label.text = '⬆️ Forward'
            ui.button('⬆️', on_click=go_forward).classes('control-btn bg-blue-600')
        
        # Left, Stop, Right
        with ui.row().classes('justify-center gap-4 mb-4'):
            def go_left():
                send_command('L')
                action_label.text = '⬅️ Left'
            ui.button('⬅️', on_click=go_left).classes('control-btn bg-blue-600')
            
            def stop():
                send_command('S')
                action_label.text = '🛑 STOPPED'
                action_label.classes(remove='text-cyan-400', add='text-red-400')
            ui.button('STOP', on_click=stop).classes('stop-btn bg-red-600 text-white font-bold')
            
            def go_right():
                send_command('R')
                action_label.text = '➡️ Right'
            ui.button('➡️', on_click=go_right).classes('control-btn bg-blue-600')
        
        # Backward
        with ui.row().classes('justify-center'):
            def go_backward():
                send_command('B')
                action_label.text = '⬇️ Backward'
            ui.button('⬇️', on_click=go_backward).classes('control-btn bg-blue-600')
    
    # Quick Commands
    with ui.card().classes('bg-slate-800/50 p-4 mt-8'):
        ui.label('Quick Commands').classes('text-white font-bold mb-2')
        with ui.row().classes('gap-2'):
            ui.button('Forward 1s', on_click=lambda: (send_command('F'), ui.timer(1.0, lambda: send_command('S'), once=True))).classes('bg-green-600')
            ui.button('Back 1s', on_click=lambda: (send_command('B'), ui.timer(1.0, lambda: send_command('S'), once=True))).classes('bg-green-600')
            ui.button('Left 0.5s', on_click=lambda: (send_command('L'), ui.timer(0.5, lambda: send_command('S'), once=True))).classes('bg-green-600')
            ui.button('Right 0.5s', on_click=lambda: (send_command('R'), ui.timer(0.5, lambda: send_command('S'), once=True))).classes('bg-green-600')
    
    # Reconnect button
    def try_reconnect():
        if connect_arduino():
            status_label.text = 'Arduino Connected ✅'
            status_label.classes(remove='text-red-400', add='text-green-400')
        else:
            status_label.text = 'Connection Failed ❌'
    
    ui.button('🔌 Reconnect Arduino', on_click=try_reconnect).classes('mt-4 bg-purple-600')
    
    # Safety hints
    ui.label('⚠️ Keep robot wheels OFF the ground for testing!').classes('text-yellow-400 mt-8')
    ui.label('💡 Press STOP immediately if anything goes wrong').classes('text-gray-500')


# Run the GUI
ui.run(title='Robot Control', port=8888, dark=True, reload=False, show=False)
