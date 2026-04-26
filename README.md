# Graduation Project: Smart Leading Robot

This project consists of multiple components that work together to provide a comprehensive object detection, navigation, and control system for a smart rescue/leading robot. The system supports **multiple robots**, **runtime AI model switching**, **real-time SLAM mapping**, **manual & autonomous navigation**, and **live sensor monitoring** — all from a single web-based dashboard.

## Installation

Before running the scripts, install the required Python libraries:

```bash
pip install opencv-python pyttsx3 ultralytics SpeechRecognition numpy streamlit PyAudio roslibpy nicegui pyzmq torch
```

## Project Structure

```
graduation/
├── arduino/                  # Arduino motor controller firmware
│   ├── Arduino code.txt
│   ├── motor_controller_v2.ino
│   └── motor_controller_v2/
├── classification/           # Camera capture & streaming scripts
│   ├── camera.py             # Standalone camera with YOLO inference
│   ├── tcp_rasp.py / tcp_lap.py
│   ├── tcp_rasp_zmq.py       # ZMQ-based camera streamer (Raspberry Pi)
│   ├── udp_rasp.py / udp_lap.py
│   └── mix_rasp.py / mix_lap.py
├── dashboard/                # Web-based dashboards
│   ├── dash.py               # Basic ROS dashboard
│   ├── dash_control.py       # Dashboard with control panel
│   ├── dash_mapping.py       # ★ Main dashboard (mapping + control + AI)
│   ├── desktop_app.py        # Tkinter desktop app
│   ├── index.py              # Streamlit dashboard
│   ├── map_debug.py          # Map debugging utility
│   └── professional_dashboard.py
├── mapping/                  # ROS2 SLAM mapping
│   ├── start_mapping.py      # Full mapping launch
│   ├── slam_only.py          # SLAM-only mode
│   ├── mapper.yaml           # slam_toolbox configuration
│   └── config.rviz           # RViz visualization config
├── models/                   # YOLO model weights
│   ├── yolov8n-fire.pt       # Dual-head fire detection model (custom ConcatHead)
│   ├── fire.pt               # Fire detection model
│   ├── yolov8n.pt            # Standard COCO detection
│   └── yolov8n-seg.pt        # Segmentation model
├── navigation/               # Robot navigation & control
│   ├── simple_explorer.py    # Autonomous exploration
│   ├── control_center.py     # Central control node
│   ├── manual_control_gui.py # Manual control GUI
│   ├── motor_controller.py   # Motor driver interface
│   ├── professional_motor_controller.py
│   ├── smart_motor_controller.py
│   ├── emergency_controller.py
│   ├── start_autonomous.py   # Autonomous navigation launcher
│   ├── fake_odom.py          # Odometry simulator for testing
│   └── nav2_params.yaml      # Nav2 navigation parameters
├── rasp_cmd/                 # Raspberry Pi automation scripts
│   ├── start_dash.sh         # Launch full dashboard system
│   ├── start_dash_rasp3.sh   # Variant for Raspberry Pi 3
│   ├── start_slam.sh         # Launch SLAM only
│   ├── start_robot.sh        # Full robot startup
│   ├── start_autonomous.sh   # Autonomous mode startup
│   ├── start_minimal.sh      # Minimal startup
│   └── start_lightweight.sh  # Lightweight startup
├── robot.nxs / robot2.nxs    # NoMachine connection profiles
└── README.md
```

## Components

### 1. Standalone Camera Classification (`classification/camera.py`)

Real-time video feed from a local camera with YOLO object detection.

**Features:**
- Real-time object detection using a custom YOLO model (`models/best.pt`)
- Visual feedback with segmentation masks and labels

**How to Run:**
```bash
python classification/camera.py
```
Press `q` on the video window to quit.

### 2. Streamlit Dashboard (`dashboard/index.py`)

A web-based dashboard using Streamlit for controlling and monitoring the camera feed.

**Features:**
- Start/Stop camera feed via buttons
- Detection confidence slider
- Voice announcements toggle
- Live detected object display

**How to Run:**
```bash
streamlit run dashboard/index.py
```

### 3. Client-Server Streaming

Video streaming from Raspberry Pi to a laptop for remote AI processing.

| Protocol | Raspberry Pi Script | Laptop Script |
|----------|-------------------|---------------|
| UDP | `classification/udp_rasp.py` | `classification/udp_lap.py` |
| TCP | `classification/tcp_rasp.py` | `classification/tcp_lap.py` |
| ZMQ | `classification/tcp_rasp_zmq.py` | — (used by dashboard) |

**How to Run:**
1. On the Raspberry Pi — update the laptop IP and run the streaming script.
2. On the Laptop — run the corresponding receiving script.

### 4. ROS-Integrated Dashboard (`dashboard/dash.py`)

Basic dashboard using `nicegui` with ROS integration for robot monitoring.

**Features:**
- Real-time connection status and logs
- Battery monitoring with live chart
- Video feed with YOLO overlays

**How to Run:**
```bash
python dashboard/dash.py
```

### 5. Mapping & Control Dashboard (`dashboard/dash_mapping.py`) ★ Main Dashboard

The primary dashboard providing comprehensive robot control, AI detection, and SLAM mapping. This is the most feature-rich component of the system.

**Features:**

- **Multi-Robot Support:** Switch between multiple robots (Alpha / Beta) at runtime via a dropdown selector. The dashboard reconnects ROS and ZMQ streams, updates the UI with the active robot's name and icon, and clears stale video frames automatically.
- **Runtime Model Switching:** Dynamically swap YOLO models from the `models/` directory without restarting. A model selector overlay on the live feed lets you pick any `.pt` file discovered at startup.
- **Dual-Head YOLO (ConcatHead):** Supports custom YOLO models trained with a `ConcatHead` module for dual-head detection (e.g., fire + general objects). The class is defined within the project and **monkey-patched** into `ultralytics.nn.modules.conv` at runtime, so no modifications to the ultralytics package are needed.
- **Real-time SLAM Map:** Renders the ROS occupancy grid map with robot pose (green circle + direction arrow) and laser scan overlay (red dots), styled exactly like RViz.
- **Live Video & AI:** Displays the ZMQ video feed annotated with the active YOLO model's detections.
- **Sensor Monitoring:** Real-time gauges for Gas Level (PPM), Battery Level (%), and Detection Confidence.
- **Manual Control:** On-screen D-Pad buttons + full keyboard support (WASD / Arrow keys / Space / Escape).
- **Autonomous Mode:** Toggle autonomous exploration on/off via the dashboard, publishing to `/explore_enable`.
- **Emergency Stop:** Dedicated emergency stop button and keyboard shortcut (Escape) with visual confirmation.
- **Speed Control:** Adjustable speed slider (0.10 – 0.30 m/s).
- **Incident Logs:** Live log stream from the robot via the `/robot_log` ROS topic.
- **Sensor History Chart:** Rolling battery level chart.

**Keyboard Controls:**

| Key | Action |
|-----|--------|
| `W` / `↑` | Move Forward |
| `S` / `↓` | Move Backward |
| `A` / `←` | Turn Left |
| `D` / `→` | Turn Right |
| `Space` | Stop |
| `Escape` | Emergency Stop |

**How to Run:**

1. **On the Raspberry Pi (Automated):**
   ```bash
   chmod +x rasp_cmd/start_dash.sh
   ./rasp_cmd/start_dash.sh
   ```
   This launches: ROS Mapping, ROS Bridge, Camera Streaming, and Map Saver.

2. **On the Laptop:**
   ```bash
   python dashboard/dash_mapping.py
   ```
   The dashboard opens at `http://localhost:8080`.

**Available Robots:**

| Hostname | Name | Icon |
|----------|------|------|
| `robot.local` | Alpha | 🤖 |
| `robot2.local` | Beta | ⚙️ |

### 6. Navigation System (`navigation/`)

ROS2-based navigation and control modules for the robot.

- **`simple_explorer.py`** — Autonomous frontier-based exploration
- **`control_center.py`** — Central control node coordinating navigation
- **`motor_controller.py`** — Low-level motor driver interface
- **`emergency_controller.py`** — Emergency stop handler
- **`manual_control_gui.py`** — Standalone manual control GUI
- **`nav2_params.yaml`** — Nav2 stack configuration

### 7. Arduino Firmware (`arduino/`)

Motor controller firmware for the Arduino, interfacing with the ROS2 system via serial communication.

- **`motor_controller_v2.ino`** — Latest motor controller with PID control

## Raspberry Pi Automation Scripts

Shell scripts in `rasp_cmd/` to automate robot startup:

| Script | Purpose |
|--------|---------|
| `start_dash.sh` | Full dashboard system (mapping + bridge + camera + map saver) |
| `start_dash_rasp3.sh` | Optimized for Raspberry Pi 3 |
| `start_slam.sh` | SLAM mapping only |
| `start_robot.sh` | Full robot system |
| `start_autonomous.sh` | Autonomous navigation mode |
| `start_minimal.sh` | Minimal startup |
| `start_lightweight.sh` | Lightweight startup |

## Custom YOLO Dual-Head (ConcatHead)

The project uses a custom `ConcatHead` module for dual-head YOLO models (e.g., combining fire detection + general object detection heads). This class is:

1. **Defined in** `dashboard/dash_mapping.py`
2. **Monkey-patched** into `ultralytics.nn.modules.conv` at runtime before model loading

This means anyone cloning the project can run it directly — **no modifications to the ultralytics package are required**.

```python
# Automatically injected at startup:
import ultralytics.nn.modules.conv as _conv_module
_conv_module.ConcatHead = ConcatHead
```

## YOLO Models

Place your `.pt` model files in the `models/` directory. The dashboard auto-discovers them at startup and lets you switch between them at runtime.

| Model | Description |
|-------|-------------|
| `yolov8n-fire.pt` | Dual-head fire detection (default, uses ConcatHead) |
| `fire.pt` | Fire detection |
| `yolov8n.pt` | Standard COCO object detection |
| `yolov8n-seg.pt` | Instance segmentation |
