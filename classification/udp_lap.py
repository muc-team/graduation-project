import cv2
import socket
import numpy as np
from ultralytics import YOLO

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 9999))
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2097152)

model = YOLO("../models/yolov8n.pt")

print("Receiving...")

while True:
    try:
        data, _ = sock.recvfrom(65536)
        
        nparr = np.frombuffer(data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if frame is not None:
            results = model(frame, verbose=False)
            annotated = results[0].plot()
            cv2.imshow('YOLO', annotated)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except Exception as e:
        continue

cv2.destroyAllWindows()
