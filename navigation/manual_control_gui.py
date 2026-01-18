#!/usr/bin/env python3
"""
Robot Manual Control GUI - Smart Version
- Hold button/key to move, release to stop
- Keyboard: W/A/S/D or Arrow Keys
- Mouse: Hold buttons to move
"""

import serial
import time
from nicegui import ui
from nicegui.events import KeyEventArguments

# ============== Serial Setup ==============
arduino = None
connected = False

def connect_arduino():
    global arduino, connected
    ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1', '/dev/ttyAMA0']
    for port in ports:
        try:
            arduino = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)
            connected = True
            print(f"✅ Connected to Arduino on {port}")
            return True
        except:
            continue
    print("❌ Could not connect to Arduino")
    return False

def send_command(cmd):
    if arduino and arduino.is_open:
        try:
            arduino.write(cmd.encode())
            return True
        except:
            pass
    return False

connect_arduino()

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
        transition: all 0.1s !important;
        cursor: pointer !important;
        user-select: none !important;
    }
    .control-btn:hover { transform: scale(1.05); }
    .control-btn:active { 
        transform: scale(0.95); 
        background: #1e40af !important;
    }
    .stop-btn {
        width: 100px !important;
        height: 100px !important;
        font-size: 24px !important;
        border-radius: 50% !important;
    }
    .key-hint {
        position: absolute;
        bottom: 5px;
        right: 10px;
        font-size: 14px;
        opacity: 0.7;
    }
</style>
''')

# Track current command to avoid spam
current_cmd = 'S'

def move(cmd, action_text):
    global current_cmd
    if current_cmd != cmd:
        send_command(cmd)
        current_cmd = cmd
        action_label.text = action_text
        if cmd == 'S':
            action_label.classes(remove='text-cyan-400 text-green-400', add='text-red-400')
        else:
            action_label.classes(remove='text-red-400', add='text-green-400')

def stop():
    move('S', '🛑 STOPPED')

with ui.column().classes('w-full items-center justify-center p-8'):
    
    ui.label('🤖 Robot Manual Control').classes('text-4xl font-bold text-white mb-2')
    ui.label('Hold to Move • Release to Stop').classes('text-gray-400 mb-2')
    ui.label('🎮 Use WASD or Arrow Keys!').classes('text-cyan-400 font-bold mb-6')
    
    # Status
    with ui.card().classes('bg-slate-800 p-4 mb-6 w-96'):
        with ui.row().classes('items-center gap-4'):
            status_icon = ui.icon('radio_button_checked' if connected else 'radio_button_unchecked', size='24px')
            status_icon.classes('text-green-400' if connected else 'text-red-400')
            status_label = ui.label('Arduino Connected ✅' if connected else 'Arduino Not Connected ❌')
            status_label.classes('font-bold text-xl ' + ('text-green-400' if connected else 'text-red-400'))
    
    # Current Action
    action_label = ui.label('Ready - Press a key or button').classes('text-2xl text-cyan-400 font-bold mb-4')
    
    # Control Pad
    with ui.card().classes('bg-slate-800/50 p-8 rounded-3xl'):
        
        # Forward - W / ↑
        with ui.row().classes('justify-center mb-4'):
            fwd_btn = ui.button('⬆️').classes('control-btn bg-blue-600')
            fwd_btn.on('mousedown', lambda: move('F', '⬆️ Forward'))
            fwd_btn.on('mouseup', stop)
            fwd_btn.on('mouseleave', stop)
            with fwd_btn:
                ui.label('W').classes('key-hint text-white')
        
        # Left, Stop, Right
        with ui.row().classes('justify-center gap-4 mb-4'):
            # Left - A / ←
            left_btn = ui.button('⬅️').classes('control-btn bg-blue-600')
            left_btn.on('mousedown', lambda: move('L', '⬅️ Left'))
            left_btn.on('mouseup', stop)
            left_btn.on('mouseleave', stop)
            with left_btn:
                ui.label('A').classes('key-hint text-white')
            
            # Stop - Space
            stop_btn = ui.button('STOP').classes('stop-btn bg-red-600 text-white font-bold')
            stop_btn.on('click', stop)
            with stop_btn:
                ui.label('Space').classes('key-hint text-white text-xs')
            
            # Right - D / →
            right_btn = ui.button('➡️').classes('control-btn bg-blue-600')
            right_btn.on('mousedown', lambda: move('R', '➡️ Right'))
            right_btn.on('mouseup', stop)
            right_btn.on('mouseleave', stop)
            with right_btn:
                ui.label('D').classes('key-hint text-white')
        
        # Backward - S / ↓
        with ui.row().classes('justify-center'):
            back_btn = ui.button('⬇️').classes('control-btn bg-blue-600')
            back_btn.on('mousedown', lambda: move('B', '⬇️ Backward'))
            back_btn.on('mouseup', stop)
            back_btn.on('mouseleave', stop)
            with back_btn:
                ui.label('S').classes('key-hint text-white')
    
    # Keyboard Controls
    ui.label('').classes('mt-6')
    with ui.card().classes('bg-slate-700/50 p-4 w-96'):
        ui.label('⌨️ Keyboard Controls').classes('text-white font-bold mb-2')
        with ui.row().classes('gap-6 justify-center'):
            with ui.column().classes('items-center'):
                ui.label('W / ↑').classes('text-cyan-400 font-mono')
                ui.label('Forward').classes('text-gray-400 text-sm')
            with ui.column().classes('items-center'):
                ui.label('A / ←').classes('text-cyan-400 font-mono')
                ui.label('Left').classes('text-gray-400 text-sm')
            with ui.column().classes('items-center'):
                ui.label('S / ↓').classes('text-cyan-400 font-mono')
                ui.label('Backward').classes('text-gray-400 text-sm')
            with ui.column().classes('items-center'):
                ui.label('D / →').classes('text-cyan-400 font-mono')
                ui.label('Right').classes('text-gray-400 text-sm')
    
    # Hints
    ui.label('⚠️ Robot wheels should be OFF the ground for first test!').classes('text-yellow-400 mt-6')

# Keyboard handler
def handle_key(e: KeyEventArguments):
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
    elif e.action.keyup:
        if e.key in ['w', 'W', 's', 'S', 'a', 'A', 'd', 'D', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']:
            stop()

ui.keyboard(on_key=handle_key)

ui.run(title='Robot Control', port=8888, dark=True, reload=False, show=False)
