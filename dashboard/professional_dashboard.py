#!/usr/bin/env python3
"""
Professional Robot Dashboard - Final
Map displays FULL SIZE, Clear works, Mapping control
"""

import cv2, zmq, time, base64, roslibpy, threading, numpy as np
from nicegui import ui
from ultralytics import YOLO

ROBOT_HOST, ROS_PORT, VIDEO_PORT = 'robot.local', 9090, 5555

class State:
    connected = False
    mapping = False
    autonomous = False
    emergency = False
    action = "Ready"
    speed = 0.2
    frame = None
    map_data = None
    frame_count = map_count = 0
    held = False
    cmd = None

S = State()
client = manual_topic = estop_topic = explore_topic = None

def init_ros():
    global client, manual_topic, estop_topic, explore_topic
    client = roslibpy.Ros(host=ROBOT_HOST, port=ROS_PORT)
    
    def ready():
        global manual_topic, estop_topic, explore_topic
        S.connected = True
        manual_topic = roslibpy.Topic(client, '/manual_cmd', 'geometry_msgs/Twist')
        manual_topic.advertise()
        estop_topic = roslibpy.Topic(client, '/emergency_stop', 'std_msgs/Bool')
        estop_topic.advertise()
        explore_topic = roslibpy.Topic(client, '/explore_enable', 'std_msgs/Bool')
        explore_topic.advertise()
        roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid').subscribe(on_map)
    
    client.on_ready(ready)
    client.on('close', lambda e: setattr(S, 'connected', False))
    threading.Thread(target=lambda: [time.sleep(2) or client.run() for _ in iter(int,1) if not client.is_connected], daemon=True).start()

def on_map(msg):
    if not S.mapping: return
    try:
        w, h = msg['info']['width'], msg['info']['height']
        data = np.array(msg['data'], dtype=np.int8).reshape((h, w))
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[data == -1] = [40, 40, 50]
        img[data == 0] = [245, 248, 250]
        img[data == 100] = [70, 70, 180]
        img = np.flipud(img)
        
        # BIG scale
        sz = 800
        scale = max(3, sz // max(w, h))
        img = cv2.resize(img, (w*scale, h*scale), interpolation=cv2.INTER_NEAREST)
        
        # Robot
        cx, cy = img.shape[1]//2, img.shape[0]//2
        cv2.circle(img, (cx, cy), 20, (50, 200, 50), -1)
        cv2.circle(img, (cx, cy), 24, (100, 255, 100), 4)
        cv2.arrowedLine(img, (cx, cy), (cx, cy-40), (100, 255, 100), 4, tipLength=0.4)
        
        _, buf = cv2.imencode('.png', img)
        S.map_data = f'data:image/png;base64,{base64.b64encode(buf).decode()}'
        S.map_count += 1
    except: pass

print("Loading YOLO...")
model = YOLO("../models/yolov8n.pt")
print("✅ Ready")

def video():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt_string(zmq.SUBSCRIBE, '')
    sock.setsockopt(zmq.RCVTIMEO, 5000)
    while True:
        try: sock.connect(f"tcp://{ROBOT_HOST}:{VIDEO_PORT}"); break
        except: time.sleep(1)
    while True:
        try:
            frame = cv2.imdecode(np.frombuffer(sock.recv(), np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                res = model(frame, verbose=False, conf=0.5)
                _, buf = cv2.imencode('.jpg', res[0].plot(), [cv2.IMWRITE_JPEG_QUALITY, 85])
                S.frame = f'data:image/jpeg;base64,{base64.b64encode(buf).decode()}'
                S.frame_count += 1
        except: time.sleep(0.1)
threading.Thread(target=video, daemon=True).start()

def send(l=0.0, a=0.0):
    if manual_topic and S.connected and not S.emergency:
        manual_topic.publish(roslibpy.Message({'linear':{'x':l,'y':0,'z':0},'angular':{'x':0,'y':0,'z':a}}))

def do(c):
    if c=='F': send(S.speed,0)
    elif c=='B': send(-S.speed,0)
    elif c=='L': send(0,0.5)
    elif c=='R': send(0,-0.5)
    else: send(0,0)

def start(c,t): S.held, S.cmd, S.action = True, c, t
def stop(): S.held, S.cmd, S.action = False, None, "Ready"; send(0,0)
def estop(): S.emergency, S.action = True, "🛑 STOP"; estop_topic and estop_topic.publish(roslibpy.Message({'data':True})); send(0,0)
def release(): S.emergency, S.action = False, "Ready"; estop_topic and estop_topic.publish(roslibpy.Message({'data':False}))
def auto(v): S.autonomous = v; explore_topic and explore_topic.publish(roslibpy.Message({'data':v})); S.action = "AUTO" if v else "Manual"
def mapping(v): S.mapping = v; S.action = "Mapping..." if v else "Map OFF"
def clear(): S.map_data = None; S.map_count = 0

threading.Thread(target=lambda: [time.sleep(0.1) or (S.held and S.cmd and do(S.cmd)) for _ in iter(int,1)], daemon=True).start()

# GUI
ui.add_head_html('<style>body{background:#0d1117}.c{background:rgba(22,27,34,.95);border:1px solid rgba(255,255,255,.1);border-radius:16px}.b{width:65px!important;height:65px!important;font-size:26px!important;border-radius:12px!important}</style>')

with ui.row().classes('w-full min-h-screen p-3 gap-3'):
    with ui.column().classes('w-64 gap-2'):
        with ui.card().classes('c p-3'):
            with ui.row().classes('items-center gap-2'):
                ui.icon('smart_toy').classes('text-blue-400')
                ui.label('RESCUE ROBOT').classes('font-bold text-white')
            with ui.row().classes('items-center gap-1 mt-1'):
                ci = ui.icon('circle', size='10px').classes('text-red-500')
                ct = ui.label('...').classes('text-xs text-gray-400')
        
        with ui.card().classes('c p-3'):
            al = ui.label('Ready').classes('text-xl font-bold text-cyan-400')
        
        with ui.card().classes('c p-4'):
            with ui.column().classes('items-center gap-1'):
                f=ui.button('▲').classes('b bg-blue-600'); f.on('mousedown',lambda:start('F','▲')); f.on('mouseup',stop); f.on('mouseleave',stop)
                with ui.row().classes('gap-1'):
                    l=ui.button('◀').classes('b bg-blue-600'); l.on('mousedown',lambda:start('L','◀')); l.on('mouseup',stop); l.on('mouseleave',stop)
                    ui.button('■',on_click=stop).classes('b bg-gray-700')
                    r=ui.button('▶').classes('b bg-blue-600'); r.on('mousedown',lambda:start('R','▶')); r.on('mouseup',stop); r.on('mouseleave',stop)
                b=ui.button('▼').classes('b bg-blue-600'); b.on('mousedown',lambda:start('B','▼')); b.on('mouseup',stop); b.on('mouseleave',stop)
            sl=ui.slider(min=0.1,max=0.5,step=0.05,value=0.2).classes('w-full mt-2'); sl.on('change',lambda:setattr(S,'speed',sl.value))
        
        with ui.card().classes('c p-2'):
            with ui.row().classes('justify-between w-full'): ui.label('Mapping').classes('text-white text-sm'); ui.switch(on_change=lambda e:mapping(e.value))
            with ui.row().classes('justify-between w-full mt-1'): ui.label('Auto').classes('text-white text-sm'); ui.switch(on_change=lambda e:auto(e.value))
        
        with ui.card().classes('c p-2'):
            with ui.row().classes('gap-1 w-full'):
                ui.button('🛑',on_click=estop).classes('flex-grow bg-red-600')
                ui.button('✓',on_click=release).classes('bg-green-600')
    
    with ui.column().classes('flex-grow gap-2'):
        with ui.card().classes('c p-2 h-full'):
            with ui.row().classes('justify-between mb-1'): ui.label('📹').classes('text-white'); fps=ui.label('--').classes('text-green-400 text-sm')
            vid=ui.image().classes('w-full rounded').style('max-height:70vh;object-fit:contain')
    
    with ui.column().classes('gap-2').style('width:850px'):
        with ui.card().classes('c p-3 h-full'):
            with ui.row().classes('justify-between items-center mb-2'):
                ui.label('🗺️ MAP').classes('text-white font-bold text-xl')
                mc=ui.label('OFF').classes('text-gray-400')
            
            # LARGE map container
            with ui.element('div').classes('w-full bg-gray-900 rounded-xl flex items-center justify-center overflow-auto').style('height:calc(100vh - 150px);min-height:700px'):
                mv=ui.image().classes('max-w-full')
                nm=ui.label('Turn ON Mapping').classes('text-gray-500 text-xl')
            
            with ui.row().classes('gap-2 mt-2 justify-center'):
                ui.button('🗑️ Clear Map',on_click=clear).classes('bg-orange-600')

def key(e):
    if e.action.keydown:
        if e.key in['w','W','ArrowUp']:start('F','▲')
        elif e.key in['s','S','ArrowDown']:start('B','▼')
        elif e.key in['a','A','ArrowLeft']:start('L','◀')
        elif e.key in['d','D','ArrowRight']:start('R','▶')
        elif e.key==' ':stop()
        elif e.key=='Escape':estop()
    elif e.action.keyup and e.key in['w','W','s','S','a','A','d','D','ArrowUp','ArrowDown','ArrowLeft','ArrowRight']:stop()
ui.keyboard(on_key=key)

lf,lt=0,time.time()
def upd():
    global lf,lt
    if S.frame:vid.source=S.frame
    if S.map_data:mv.source=S.map_data;mc.text=f'{S.map_count}';mc.classes(remove='text-gray-400',add='text-green-400');nm.set_visibility(False)
    else:nm.set_visibility(True);mc.text='OFF' if not S.mapping else '...'
    ci.classes(remove='text-red-500'if S.connected else'text-green-500',add='text-green-500'if S.connected else'text-red-500')
    ct.text='OK'if S.connected else'...'
    al.text=S.action
    n=time.time()
    if n-lt>=1:fps.text=f'{S.frame_count-lf}';lf,lt=S.frame_count,n
ui.timer(0.1,upd)

init_ros()
ui.run(title='Robot',port=8080,dark=True,reload=False)
