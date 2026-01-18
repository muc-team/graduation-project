#!/usr/bin/env python3
"""
Robot Control - Tkinter Desktop App
Built-in Python GUI - no extra dependencies
"""

import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import zmq
import time
import threading
import numpy as np
import roslibpy
from ultralytics import YOLO

ROBOT = 'robot.local'


class RobotApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Rescue Robot Control")
        self.root.configure(bg='#0d1117')
        self.root.state('zoomed')  # Maximized
        
        # State
        self.connected = False
        self.mapping = tk.BooleanVar(value=False)
        self.autonomous = tk.BooleanVar(value=False)
        self.emergency = False
        self.speed = 0.15
        self.held_key = None
        self.video_frame = None
        self.map_frame = None
        self.fps = 0
        self.action = "Ready"
        
        # ROS
        self.ros = None
        self.manual_topic = None
        self.estop_topic = None
        self.explore_topic = None
        
        # YOLO
        print("Loading YOLO...")
        self.yolo = YOLO("../models/yolov8n.pt")
        print("✅ YOLO Ready")
        
        self.setup_ui()
        self.bind_keys()
        self.start_threads()
        
        # Update loop
        self.update_display()
        self.command_loop()
        
        self.root.mainloop()
    
    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#0d1117')
        style.configure('TLabel', background='#0d1117', foreground='white')
        style.configure('TButton', padding=10)
        style.configure('TCheckbutton', background='#0d1117', foreground='white')
        
        main = ttk.Frame(self.root)
        main.pack(fill='both', expand=True, padx=10, pady=10)
        
        # LEFT Panel
        left = ttk.Frame(main)
        left.pack(side='left', fill='y', padx=(0, 10))
        
        # Status
        status_frame = ttk.LabelFrame(left, text="STATUS", padding=10)
        status_frame.pack(fill='x', pady=(0, 10))
        
        self.status_label = ttk.Label(status_frame, text="Ready", font=('Arial', 18, 'bold'))
        self.status_label.pack()
        
        self.conn_label = ttk.Label(status_frame, text="● Connecting...", foreground='red')
        self.conn_label.pack()
        
        # Controls
        ctrl_frame = ttk.LabelFrame(left, text="CONTROLS", padding=10)
        ctrl_frame.pack(fill='x', pady=(0, 10))
        
        btn_style = {'width': 5, 'font': ('Arial', 16)}
        
        self.btn_f = tk.Button(ctrl_frame, text="▲", bg='#1f6feb', fg='white', **btn_style)
        self.btn_f.grid(row=0, column=1, padx=2, pady=2)
        
        self.btn_l = tk.Button(ctrl_frame, text="◀", bg='#1f6feb', fg='white', **btn_style)
        self.btn_l.grid(row=1, column=0, padx=2, pady=2)
        
        self.btn_s = tk.Button(ctrl_frame, text="■", bg='#484f58', fg='white', **btn_style, command=self.stop)
        self.btn_s.grid(row=1, column=1, padx=2, pady=2)
        
        self.btn_r = tk.Button(ctrl_frame, text="▶", bg='#1f6feb', fg='white', **btn_style)
        self.btn_r.grid(row=1, column=2, padx=2, pady=2)
        
        self.btn_b = tk.Button(ctrl_frame, text="▼", bg='#1f6feb', fg='white', **btn_style)
        self.btn_b.grid(row=2, column=1, padx=2, pady=2)
        
        # Button bindings
        for btn, key in [(self.btn_f, 'F'), (self.btn_l, 'L'), (self.btn_r, 'R'), (self.btn_b, 'B')]:
            btn.bind('<ButtonPress-1>', lambda e, k=key: self.key_press(k))
            btn.bind('<ButtonRelease-1>', lambda e: self.key_release())
        
        # Speed
        speed_frame = ttk.LabelFrame(left, text="SPEED", padding=10)
        speed_frame.pack(fill='x', pady=(0, 10))
        
        self.speed_slider = ttk.Scale(speed_frame, from_=0.1, to=0.3, value=0.15, 
                                       command=lambda v: setattr(self, 'speed', float(v)))
        self.speed_slider.pack(fill='x')
        self.speed_label = ttk.Label(speed_frame, text="0.15 m/s")
        self.speed_label.pack()
        
        # Modes
        mode_frame = ttk.LabelFrame(left, text="MODES", padding=10)
        mode_frame.pack(fill='x', pady=(0, 10))
        
        ttk.Checkbutton(mode_frame, text="Mapping", variable=self.mapping).pack(anchor='w')
        ttk.Checkbutton(mode_frame, text="Autonomous", variable=self.autonomous,
                        command=self.toggle_autonomous).pack(anchor='w')
        
        # Emergency
        emerg_frame = ttk.LabelFrame(left, text="EMERGENCY", padding=10)
        emerg_frame.pack(fill='x')
        
        tk.Button(emerg_frame, text="🛑 STOP", bg='#da3633', fg='white', font=('Arial', 12, 'bold'),
                  command=self.emergency_stop).pack(side='left', expand=True, fill='x', padx=(0, 5))
        tk.Button(emerg_frame, text="✓", bg='#238636', fg='white', font=('Arial', 12),
                  command=self.release_emergency).pack(side='left')
        
        # CENTER: Video
        center = ttk.Frame(main)
        center.pack(side='left', fill='both', expand=True, padx=(0, 10))
        
        video_frame = ttk.LabelFrame(center, text="LIVE CAMERA + YOLO", padding=5)
        video_frame.pack(fill='both', expand=True)
        
        self.video_canvas = tk.Canvas(video_frame, bg='#161b22', highlightthickness=0)
        self.video_canvas.pack(fill='both', expand=True)
        
        self.fps_label_vid = ttk.Label(video_frame, text="-- FPS", foreground='#3fb950')
        self.fps_label_vid.pack()
        
        # RIGHT: Map
        right = ttk.Frame(main)
        right.pack(side='left', fill='both', expand=True)
        
        map_frame = ttk.LabelFrame(right, text="SLAM MAP", padding=5)
        map_frame.pack(fill='both', expand=True)
        
        self.map_canvas = tk.Canvas(map_frame, bg='#161b22', highlightthickness=0)
        self.map_canvas.pack(fill='both', expand=True)
        
        self.map_text = self.map_canvas.create_text(250, 200, text="Enable Mapping", 
                                                     fill='#484f58', font=('Arial', 14))
        
        tk.Button(map_frame, text="🗑️ Clear", bg='#f0883e', fg='white',
                  command=self.clear_map).pack(pady=5)
    
    def bind_keys(self):
        self.root.bind('<KeyPress-w>', lambda e: self.key_press('F'))
        self.root.bind('<KeyPress-W>', lambda e: self.key_press('F'))
        self.root.bind('<KeyPress-Up>', lambda e: self.key_press('F'))
        
        self.root.bind('<KeyPress-s>', lambda e: self.key_press('B'))
        self.root.bind('<KeyPress-S>', lambda e: self.key_press('B'))
        self.root.bind('<KeyPress-Down>', lambda e: self.key_press('B'))
        
        self.root.bind('<KeyPress-a>', lambda e: self.key_press('L'))
        self.root.bind('<KeyPress-A>', lambda e: self.key_press('L'))
        self.root.bind('<KeyPress-Left>', lambda e: self.key_press('L'))
        
        self.root.bind('<KeyPress-d>', lambda e: self.key_press('R'))
        self.root.bind('<KeyPress-D>', lambda e: self.key_press('R'))
        self.root.bind('<KeyPress-Right>', lambda e: self.key_press('R'))
        
        self.root.bind('<KeyRelease-w>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-W>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-s>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-S>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-a>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-A>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-d>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-D>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-Up>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-Down>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-Left>', lambda e: self.key_release())
        self.root.bind('<KeyRelease-Right>', lambda e: self.key_release())
        
        self.root.bind('<space>', lambda e: self.stop())
        self.root.bind('<Escape>', lambda e: self.emergency_stop())
    
    def start_threads(self):
        threading.Thread(target=self.ros_thread, daemon=True).start()
        threading.Thread(target=self.video_thread, daemon=True).start()
    
    def ros_thread(self):
        while True:
            try:
                self.ros = roslibpy.Ros(host=ROBOT, port=9090)
                self.ros.on_ready(self.on_ros_ready)
                self.ros.run()
            except Exception as e:
                print(f"ROS error: {e}")
                self.connected = False
            time.sleep(3)
    
    def on_ros_ready(self):
        print("✅ ROS Connected")
        self.connected = True
        
        self.manual_topic = roslibpy.Topic(self.ros, '/manual_cmd', 'geometry_msgs/Twist')
        self.manual_topic.advertise()
        
        self.estop_topic = roslibpy.Topic(self.ros, '/emergency_stop', 'std_msgs/Bool')
        self.estop_topic.advertise()
        
        self.explore_topic = roslibpy.Topic(self.ros, '/explore_enable', 'std_msgs/Bool')
        self.explore_topic.advertise()
        
        map_topic = roslibpy.Topic(self.ros, '/map', 'nav_msgs/OccupancyGrid')
        map_topic.subscribe(self.on_map)
    
    def on_map(self, msg):
        if not self.mapping.get():
            return
        try:
            w = msg['info']['width']
            h = msg['info']['height']
            data = np.array(msg['data'], dtype=np.int8).reshape((h, w))
            
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[data == -1] = [40, 40, 50]
            img[data == 0] = [230, 235, 240]
            img[data == 100] = [60, 60, 180]
            
            img = np.flipud(img)
            
            scale = max(2, 500 // max(w, h))
            img = cv2.resize(img, (w*scale, h*scale), interpolation=cv2.INTER_NEAREST)
            
            # Robot
            cx, cy = img.shape[1]//2, img.shape[0]//2
            cv2.rectangle(img, (cx-15, cy-20), (cx+15, cy+20), (0, 180, 0), -1)
            cv2.rectangle(img, (cx-15, cy-20), (cx+15, cy+20), (0, 255, 0), 2)
            pts = np.array([[cx, cy-30], [cx-10, cy-15], [cx+10, cy-15]], np.int32)
            cv2.fillPoly(img, [pts], (0, 255, 255))
            
            self.map_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except Exception as e:
            print(f"Map error: {e}")
    
    def video_thread(self):
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
        
        fc = 0
        lt = time.time()
        
        while True:
            try:
                data = sock.recv()
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    results = self.yolo(frame, verbose=False, conf=0.5)
                    annotated = results[0].plot()
                    self.video_frame = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
                    fc += 1
                    
                    now = time.time()
                    if now - lt >= 1:
                        self.fps = fc
                        fc = 0
                        lt = now
            except:
                time.sleep(0.05)
    
    def send_twist(self, linear, angular):
        if self.manual_topic and self.connected and not self.emergency:
            self.manual_topic.publish(roslibpy.Message({
                'linear': {'x': linear, 'y': 0, 'z': 0},
                'angular': {'x': 0, 'y': 0, 'z': angular}
            }))
    
    def key_press(self, key):
        self.held_key = key
        self.action = f"Moving {key}"
    
    def key_release(self):
        self.held_key = None
        self.action = "Stopped"
        self.send_twist(0, 0)
    
    def stop(self):
        self.held_key = None
        self.action = "Stopped"
        self.send_twist(0, 0)
    
    def emergency_stop(self):
        self.emergency = True
        self.action = "🛑 EMERGENCY"
        if self.estop_topic:
            self.estop_topic.publish(roslibpy.Message({'data': True}))
        self.send_twist(0, 0)
    
    def release_emergency(self):
        self.emergency = False
        self.action = "Ready"
        if self.estop_topic:
            self.estop_topic.publish(roslibpy.Message({'data': False}))
    
    def toggle_autonomous(self):
        if self.explore_topic:
            self.explore_topic.publish(roslibpy.Message({'data': self.autonomous.get()}))
        self.action = "AUTO" if self.autonomous.get() else "Manual"
    
    def clear_map(self):
        self.map_frame = None
    
    def command_loop(self):
        if self.held_key:
            if self.held_key == 'F':
                self.send_twist(self.speed, 0)
            elif self.held_key == 'B':
                self.send_twist(-self.speed, 0)
            elif self.held_key == 'L':
                self.send_twist(0, 0.5)
            elif self.held_key == 'R':
                self.send_twist(0, -0.5)
        
        self.root.after(80, self.command_loop)
    
    def update_display(self):
        # Update status
        self.status_label.config(text=self.action)
        if self.connected:
            self.conn_label.config(text="● Connected", foreground='#3fb950')
        else:
            self.conn_label.config(text="● Disconnected", foreground='red')
        
        self.fps_label_vid.config(text=f"{self.fps} FPS")
        self.speed_label.config(text=f"{self.speed:.2f} m/s")
        
        # Update video
        if self.video_frame is not None:
            self.show_frame(self.video_canvas, self.video_frame)
        
        # Update map
        if self.map_frame is not None:
            self.map_canvas.delete(self.map_text)
            self.show_frame(self.map_canvas, self.map_frame)
        
        self.root.after(100, self.update_display)
    
    def show_frame(self, canvas, frame):
        cw = canvas.winfo_width()
        ch = canvas.winfo_height()
        if cw <= 1 or ch <= 1:
            return
        
        h, w = frame.shape[:2]
        scale = min(cw/w, ch/h)
        new_w, new_h = int(w*scale), int(h*scale)
        
        resized = cv2.resize(frame, (new_w, new_h))
        img = Image.fromarray(resized)
        photo = ImageTk.PhotoImage(img)
        
        canvas.delete("all")
        canvas.create_image(cw//2, ch//2, image=photo)
        canvas._photo = photo  # Keep reference


if __name__ == '__main__':
    print("Starting Robot Control...")
    RobotApp()
