import cv2
import time
import struct
import socket
import numpy as np
from ultralytics import YOLO
from collections import defaultdict

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 9999))
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4194304)
sock.settimeout(0.05)

model = YOLO("../models/yolov8n.pt")

frames = defaultdict(dict)
last_processed_frame = -1
last_receive_time = time.time()

print("Receiver ready...")

while True:
    try:
        packet, _ = sock.recvfrom(65536)
        last_receive_time = time.time()
        
        frame_id, chunk_id, num_chunks = struct.unpack('!III', packet[:12])
        chunk_data = packet[12:]
        
        frames[frame_id][chunk_id] = chunk_data
        
        if len(frames[frame_id]) == num_chunks:
            if frame_id > last_processed_frame:
                
                data = b''.join([frames[frame_id][i] for i in range(num_chunks)])
                
                nparr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    results = model(frame, verbose=False)
                    annotated = results[0].plot()
                    cv2.imshow('YOLO', annotated)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
                last_processed_frame = frame_id
            
            old_frames = [fid for fid in frames if fid <= frame_id]
            for fid in old_frames:
                del frames[fid]
        
        if time.time() - last_receive_time > 1.0:
            frames.clear()
            last_receive_time = time.time()
            
    except socket.timeout:
        if time.time() - last_receive_time > 2.0:
            print("Connection lost, waiting...")
            frames.clear()
            last_receive_time = time.time()
        continue
        
    except Exception as e:
        continue

cv2.destroyAllWindows()