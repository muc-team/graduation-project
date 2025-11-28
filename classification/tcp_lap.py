import cv2
import zmq
import numpy as np
from ultralytics import YOLO

model = YOLO("../models/yolov8n.pt")
ip_address = "192.168.1.18"
port = 5555

context = zmq.Context()
socket = context.socket(zmq.SUB)

socket.setsockopt(zmq.CONFLATE, 1)

socket.setsockopt_string(zmq.SUBSCRIBE, '')

socket.connect(f"tcp://{ip_address}:{port}")

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