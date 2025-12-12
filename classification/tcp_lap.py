import cv2
import zmq
import numpy as np
from ultralytics import YOLO

model = YOLO("../models/yolov8n.pt")
RASPBERRY_IP = "192.168.1.200"
PORT = 5555

context = zmq.Context()
socket = context.socket(zmq.SUB)

socket.setsockopt(zmq.CONFLATE, 1)

socket.setsockopt_string(zmq.SUBSCRIBE, '')

socket.connect(f"tcp://{RASPBERRY_IP}:{PORT}")

print("Waiting for images...")

while True:
    jpg_as_bytes = socket.recv()
    
    image_buffer = np.frombuffer(jpg_as_bytes, dtype=np.uint8)
    frame = cv2.imdecode(image_buffer, cv2.IMREAD_COLOR)

    results = model(frame, verbose=False)
    annotated_frame = results[0].plot()

    cv2.imshow("ZeroMQ YOLO Stream", annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()