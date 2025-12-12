import cv2
import zmq
import time
import base64
import roslibpy
import threading
import numpy as np
from nicegui import ui, app
from ultralytics import YOLO
RASPBERRY_IP = '192.168.1.200'
ROS_PORT, TCP_PORT = 9090, 5555

model = YOLO("../models/yolov8n.pt") 
client = roslibpy.Ros(host=RASPBERRY_IP, port=ROS_PORT)

status_label = None
video_image = None
map_image = None
battery_chart = None
gas_knob = None
battery_knob = None
confidence_knob = None
log_container = None
connection_notified = False

latest_frame_b64 = None 
latest_map_b64 = None 

frame_counter = 0        
ui_frame_counter = 0     
map_counter = 0          
ui_map_counter = 0       

def map_callback(msg):
    global latest_map_b64, map_counter
    try:
        width = msg['info']['width']
        height = msg['info']['height']
        raw_data = msg['data']
        
        data = np.array(raw_data, dtype=np.int8).reshape((height, width))
        
        img = np.full((height, width, 3), 20, dtype=np.uint8) 
        img[data == 0] = [255, 255, 255]
        img[data == 100] = [255, 50, 50]
        
        img = np.flipud(img) 
        
        _, buffer = cv2.imencode('.png', img)
        b64_str = base64.b64encode(buffer).decode('utf-8')
        
        latest_map_b64 = f'data:image/png;base64,{b64_str}'
        
        map_counter += 1
        
    except Exception as e:
        pass

map_listener = roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
map_listener.subscribe(map_callback)

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
    while True:
        try:
            if not client.is_connected:
                client.run()
            time.sleep(2)
        except:
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

@ui.page('/')
def main_page():
    global status_label, battery_chart, video_image, map_image, gas_knob, battery_knob, confidence_knob, log_container
    
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
            
            with ui.card().classes('glass-card w-full h-1/2 p-0 relative bg-gray-900 border-2 border-green-900'):
                ui.label('SLAM MAP').classes('absolute top-3 left-3 z-10 text-black bg-white px-2 py-0.5 text-xs rounded font-bold shadow-lg')
                map_image = ui.interactive_image().classes('w-full h-full object-contain')

            with ui.column().classes('w-full flex-grow gap-3'):
                def btn_style(color): return f'w-full h-12 text-white font-bold rounded-xl shadow-lg bg-gradient-to-r from-{color}-600 to-{color}-800 hover:scale-105 transition'
                
                ui.button('START MISSION', icon='play_arrow').classes(btn_style('cyan'))
                ui.button('MANUAL CONTROL', icon='gamepad').classes(btn_style('blue'))
                ui.button('SEND TO POINT', icon='gps_fixed').classes(btn_style('indigo'))
                ui.button('EMERGENCY STOP', icon='warning').classes('w-full h-14 text-white font-bold rounded-xl shadow-lg bg-red-600 border-2 border-red-400 hover:bg-red-700 transition')

    ui.timer(1.0, update_connection_status)
    ui.timer(0.05, update_ui_content)

if __name__ in {"__main__", "__mp_main__"}:
    t1 = threading.Thread(target=connect_to_ros_thread, daemon=True)
    t1.start()
    t2 = threading.Thread(target=video_stream_loop, daemon=True)
    t2.start()
    
    ui.run(title='Sci-Fi Robot Dashboard', dark=True, port=8080, reload=False)