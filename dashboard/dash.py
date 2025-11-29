import base64
import cv2
import numpy as np
import socket
import threading
import roslibpy
import time
from nicegui import ui, app
from ultralytics import YOLO

RASPBERRY_IP_ROS = '192.168.1.19'
ROS_PORT = 9090
UDP_PORT = 9999

model = YOLO("../models/yolov8n.pt")
client = roslibpy.Ros(host=RASPBERRY_IP_ROS, port=ROS_PORT)

status_label = None
video_image = None
log_container = None
battery_chart = None
connection_notified = False

latest_frame_b64 = None 
frame_counter = 0        
ui_frame_counter = 0     

def connect_to_ros_thread():
    while True:
        try:
            if not client.is_connected:
                client.run()
            time.sleep(2)
        except:
            time.sleep(2)

def update_connection_status():
    global connection_notified, status_label
    if status_label and client.is_connected:
        status_label.text = 'Master Robot: Online'
        status_label.classes(remove='text-red-400', add='text-green-400')
        
        if not connection_notified:
            ui.notify('Connected to Robot (ROS)!', type='positive')
            connection_notified = True

def log_callback(message):
    if log_container: log_container.push(message['data'])

def battery_callback(message):
    if battery_chart:
        val = message.get('percentage', 0)
        battery_chart.options['series'][0]['data'].append(val)
        if len(battery_chart.options['series'][0]['data']) > 20:
            battery_chart.options['series'][0]['data'].pop(0)
        battery_chart.update()

listener = roslibpy.Topic(client, '/robot_log', 'std_msgs/String')
listener.subscribe(log_callback)
battery_listener = roslibpy.Topic(client, '/battery_state', 'sensor_msgs/BatteryState')
battery_listener.subscribe(battery_callback)

def video_stream_loop():
    global latest_frame_b64, frame_counter
    print("🎥 Video Thread Initializing...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2097152)
    
    bound = False
    while not bound:
        try:
            sock.bind(('0.0.0.0', UDP_PORT))
            print(f"✅ Socket Bound to {UDP_PORT}")
            bound = True
        except OSError:
            print(f"⚠️ Port {UDP_PORT} busy. Retrying...")
            time.sleep(1)

    print("🎥 Receiving Packets...")
    
    while True:
        try:
            data, _ = sock.recvfrom(65536)
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                results = model(frame, verbose=False)
                annotated_frame = results[0].plot()

                _, buffer = cv2.imencode('.jpg', annotated_frame)
                b64_string = base64.b64encode(buffer).decode('utf-8')
                
                latest_frame_b64 = f'data:image/jpeg;base64,{b64_string}'
                frame_counter += 1
                
        except Exception as e:
            continue

def update_ui_video():
    global ui_frame_counter
    if video_image and latest_frame_b64 and (frame_counter > ui_frame_counter):
        video_image.set_source(latest_frame_b64)
        ui_frame_counter = frame_counter

@ui.page('/')
def main_page():
    global status_label, log_container, battery_chart, video_image
    
    ui.dark_mode().enable()

    card_style = '''
        background-color: #1e1e2e; border-radius: 15px; padding: 20px; 
        box-shadow: 0 4px 6px rgba(0,0,0,0.3); border: 1px solid #333;
        display: flex; flex-direction: column;
    '''

    with ui.row().classes('w-full h-screen bg-slate-900 p-4 gap-4 no-wrap items-stretch'):
        
        with ui.column().classes('w-1/3 h-full gap-4'):
            with ui.card().style(card_style).classes('w-full'):
                ui.label('System Status').classes('text-xl text-blue-400 font-bold mb-2')
                with ui.row().classes('items-center'):
                    ui.icon('smart_toy', size='32px').classes('text-green-400')
                    status_label = ui.label('Master Robot: Connecting...').classes('text-red-400 font-bold')
                ui.separator().classes('my-2 bg-gray-700')
                with ui.row().classes('items-center'):
                    ui.icon('wifi', size='24px').classes('text-gray-400')
                    ui.label(f'{RASPBERRY_IP_ROS}:{UDP_PORT}').classes('text-gray-400 text-sm')

            with ui.card().style(card_style).classes('w-full flex-grow'):
                ui.label('Incident Report / Logs').classes('text-lg text-red-400 font-bold')
                log_container = ui.log().classes('w-full h-full bg-slate-800 rounded p-2 text-sm text-green-300 font-mono')
                ui.button('EXPORT PDF', icon='picture_as_pdf', on_click=lambda: ui.notify('Report Generated')).classes('bg-cyan-600 w-full mt-4')

        with ui.column().classes('w-2/3 h-full gap-4'):
            with ui.card().style(card_style).classes('w-full h-2/3 p-0 overflow-hidden bg-black relative group'):
                ui.label('YOLO LIVE FEED').classes('absolute top-4 left-4 z-10 text-white bg-red-600 px-2 rounded font-bold')
                video_image = ui.interactive_image().classes('w-full h-full object-contain')

            with ui.card().style(card_style).classes('w-full h-1/3'):
                ui.label('Sensor Metrics').classes('text-lg text-orange-400 font-bold')
                battery_chart = ui.echart({
                    'grid': {'top': '15%', 'bottom': '15%', 'left': '5%', 'right': '5%'},
                    'tooltip': {'trigger': 'axis'},
                    'xAxis': {'type': 'category', 'boundaryGap': False, 'data': [str(i) for i in range(20)]},
                    'yAxis': {'type': 'value', 'min': 0, 'max': 100, 'splitLine': {'lineStyle': {'color': '#333'}}},
                    'series': [{'name': 'Battery', 'data': [0]*20, 'type': 'line', 'smooth': True, 'color': '#f39c12'}]
                }).classes('w-full h-full')

    ui.timer(1.0, update_connection_status)
    
    ui.timer(0.03, update_ui_video)

if __name__ in {"__main__", "__mp_main__"}:
    t1 = threading.Thread(target=connect_to_ros_thread, daemon=True)
    t1.start()
    
    t2 = threading.Thread(target=video_stream_loop, daemon=True)
    t2.start()

    ui.run(title='YOLO Robot Dashboard', dark=True, port=8080, reload=False)