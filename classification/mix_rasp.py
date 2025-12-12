import cv2
import time
import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4194304)

LAPTOP_IP = "192.168.1.201"
PORT = 9999

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
frame_id = 0

print("Streaming started...")

while True:
    ret, frame = cap.read()
    if not ret:
        time.sleep(0.001)
        continue
    
    try:
        _, buffer = cv2.imencode('.jpg', frame, encode_param)
        data = buffer.tobytes()
        
        total_size = len(data)
        chunk_size = 50000
        num_chunks = (total_size + chunk_size - 1) // chunk_size
        
        for i in range(num_chunks):
            chunk = data[i*chunk_size:(i+1)*chunk_size]
            header = struct.pack('!III', frame_id, i, num_chunks)
            sock.sendto(header + chunk, (LAPTOP_IP, PORT))
        
        frame_id = (frame_id + 1) % 65536
        
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(0.01)
        continue

cap.release()
