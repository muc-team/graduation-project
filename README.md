# Graduation Project: Smart Leading Robot

## Installation

Before running the scripts, you need to install the required Python libraries. You can install them using pip:

```bash
pip install opencv-python pyttsx3 ultralytics SpeechRecognition numpy streamlit PyAudio
```

## Components

### 1. Standalone Camera Classification (`classification/camera.py`)

This script provides a direct, real-time video feed from the default camera. It uses voice commands to control the object detection process.

**Features:**
-   **Real-time Object Detection:** Captures video from the camera and uses the YOLOv8 model to detect and segment objects in real-time.
-   **Voice Control:** A voice listener runs in the background.
    -   Say **"go"** to start the object detection.
    -   Say **"stop"** to pause the detection.
-   **Voice Announcements:** When an object is detected for the first time, the system announces it using text-to-speech (e.g., "I found a person").
-   **Visual Feedback:** The video feed displays the segmentation masks and labels for detected objects. The current status (PROCESSING or PAUSED) is also shown on the screen.

**How to Run:**
```bash
python classification/camera.py
```
Press 'q' on the video window to quit.

### 2. Streamlit Dashboard (`dashboard/index.py`)

This script launches a web-based dashboard using Streamlit, providing a more user-friendly interface to control and monitor the rescue robot's camera feed.

**Features:**
-   **Interactive Web UI:** A clean and simple dashboard to view the live camera feed.
-   **Control Panel:**
    -   **Start/Stop Buttons:** Easily start and stop the camera feed.
    -   **Detection Confidence Slider:** Adjust the confidence threshold for object detection to filter out less certain detections.
    -   **Voice Announcements Toggle:** Enable or disable the text-to-speech announcements.
-   **Live Data:** Displays the name of the most recently detected object.
-   **Voice Control:** Also supports "go" and "stop" voice commands to control processing, similar to the standalone script.

**How to Run:**
To run the dashboard, execute the following command in your terminal:
```bash
streamlit run dashboard/index.py
