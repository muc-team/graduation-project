# Graduation Project: Smart Leading Robot

This project consists of four main components that work together to provide a comprehensive object detection and navigation system for a smart leading robot. The system can be run in different modes: a standalone camera classification script, a web-based Streamlit dashboard, a client-server streaming setup for remote processing, and a ROS-integrated dashboard for advanced control and monitoring.

## Installation

Before running the scripts, you need to install the required Python libraries. You can install them using pip:

```bash
pip install opencv-python pyttsx3 ultralytics SpeechRecognition numpy streamlit PyAudio roslibpy nicegui
```

## Components

### 1. Standalone Camera Classification (`classification/camera.py`)

This script provides a direct, real-time video feed from the default camera. It uses voice commands to control the object detection process.

**Features:**

- **Real-time Object Detection:** Captures video from the camera and uses the YOLOv8 model to detect and segment objects in real-time.
- **Voice Control:** A voice listener runs in the background.
  - Say **"go"** to start the object detection.
  - Say **"stop"** to pause the detection.
- **Voice Announcements:** When an object is detected for the first time, the system announces it using text-to-speech (e.g., "I found a person").
- **Visual Feedback:** The video feed displays the segmentation masks and labels for detected objects. The current status (PROCESSING or PAUSED) is also shown on the screen.

**How to Run:**

```bash
python classification/camera.py
```

Press 'q' on the video window to quit.

### 2. Streamlit Dashboard (`dashboard/index.py`)

This script launches a web-based dashboard using Streamlit, providing a more user-friendly interface to control and monitor the rescue robot's camera feed.

**Features:**

- **Interactive Web UI:** A clean and simple dashboard to view the live camera feed.
- **Control Panel:**
  - **Start/Stop Buttons:** Easily start and stop the camera feed.
  - **Detection Confidence Slider:** Adjust the confidence threshold for object detection to filter out less certain detections.
  - **Voice Announcements Toggle:** Enable or disable the text-to-speech announcements.
- **Live Data:** Displays the name of the most recently detected object.
- **Voice Control:** Also supports "go" and "stop" voice commands to control processing, similar to the standalone script.

**How to Run:**
To run the dashboard, execute the following command in your terminal:

```bash
streamlit run dashboard/index.py
```

### 3. Client-Server Streaming (`classification/udp_rasp.py` and `classification/udp_lap.py`)

This component allows for a client-server setup where a Raspberry Pi captures video and streams it to a laptop for processing. This is useful for offloading the heavy processing from the Raspberry Pi.

**Features:**

- **Remote Processing:** The Raspberry Pi (`udp_rasp.py`) captures video, encodes it, and sends it over the network to the laptop.
- **Real-time Object Detection:** The laptop (`udp_lap.py`) receives the video stream, decodes it, and runs the YOLOv8 model for object detection.
- **Visual Feedback:** The laptop displays the video feed with the segmentation masks and labels for detected objects.

**How to Run:**

_Recommended: Use the automated script (see below)._

1.  **On the Raspberry Pi:**

    - Update the `LAPTOP_IP` variable in `classification/udp_rasp.py` to the IP address of your laptop.
    - Run the script:
      ```bash
      python classification/udp_rasp.py
      ```

2.  **On the Laptop:**
    - Run the script:
      ```bash
      python classification/udp_lap.py
      ```

Press 'q' on the video window to quit.

### 4. ROS-Integrated Dashboard (`dashboard/dash.py` and `classification/udp_rasp.py`)

This component provides an advanced dashboard using `nicegui` that integrates with ROS (Robot Operating System) for more comprehensive robot monitoring and control.

**Features:**

- **Advanced UI:** A modern, real-time dashboard with status indicators, logs, and sensor metrics.
- **ROS Integration:** Connects to a ROS master on the robot to receive log messages and battery status.
- **High-Performance Video:** Uses a UDP-based protocol for streaming video from the Raspberry Pi to the dashboard.
- **Live Monitoring:** Includes widgets for system status, incident reports/logs, and a live chart for sensor data like battery percentage.

**How to Run:**

_Recommended: Use the automated script (see below)._

1.  **On the Raspberry Pi:**

    - First, start the ROS bridge to allow communication. Open a terminal and run:
      ```bash
      ros2 launch rosbridge_server rosbridge_websocket_launch.xml
      ```
    - In a second terminal, start the video streaming script. Make sure to update the `LAPTOP_IP` variable in `classification/udp_rasp.py` to your laptop's IP address.
      ```bash
      python classification/udp_rasp.py
      ```

2.  **On the Laptop:**
    - Make sure the `RASPBERRY_IP_ROS` variable in `dashboard/dash.py` is set to your Raspberry Pi's IP address.
    - Run the dashboard script:
      ```bash
      python dashboard/dash.py
      ```
    - Open your web browser and navigate to `http://localhost:8080`.

### 5. Mapping & Control Dashboard (`dashboard/dash_mapping.py`)

This component provides a cutting-edge, sci-fi styled dashboard for comprehensive robot control and mapping. It integrates real-time SLAM mapping, video feed with object detection, and sensor monitoring (gas, battery).

**Features:**

- **Real-time SLAM Map:** Visualizes the robot's environment using ROS occupancy grid maps.
- **Live Video & AI:** Displays video feed with YOLOv8 object detection overlays.
- **Sensor Monitoring:** Real-time gauges for Gas Level (PPM) and Battery Level (%).
- **Interactive Controls:** Buttons for Mission Start, Manual Control, Navigation Points, and Emergency Stop.
- **Status & Logs:** Live connection status and incident reporting logs.

**How to Run:**

1.  **On the Raspberry Pi (Automated):**

    - Navigate to the `rasp_cmd` directory.
    - Make the script executable: `chmod +x start_dash.sh`
    - Run the start script (requires GUI/GNOME terminal):
      ```bash
      ./rasp_cmd/start_dash.sh
      ```
    - This will automatically launch:
      - ROS Mapping System
      - ROS Bridge (for WebSocket communication)
      - Camera Streaming Script
      - Map Saver Utility

2.  **On the Laptop:**
    - Update `RASPBERRY_IP` in `dashboard/dash_mapping.py` to your Raspberry Pi's IP.
    - Run the dashboard:
      ```bash
      python dashboard/dash_mapping.py
      ```
    - The dashboard will automatically open in your browser (default port 8080).

## Raspberry Pi Automation Scripts

For convenience, the `rasp_cmd` directory contains shell scripts to automate the startup process on the Raspberry Pi.

### `start_dash.sh`

This is the main startup script for the full system. It launches four separate terminal tabs:

1.  **MAPPING SYSTEM:** Starts the ROS2 mapping launch file.
2.  **ROS BRIDGE:** Starts the websocket server to connect with the dashboard.
3.  **CAMERA STREAM:** Runs `udp_rasp.py` to stream video to the laptop.
4.  **SAVE MAP UTILITY:** A helper tab with the command to save the map pre-typed.

**Usage:**

```bash
cd rasp_cmd
chmod +x start_dash.sh
./start_dash.sh
```

### `start_slam.sh`

Use this script if you only want to run the SLAM mapping without the full dashboard integration.
**Usage:**

```bash
cd rasp_cmd
chmod +x start_slam.sh
./start_slam.sh
```
