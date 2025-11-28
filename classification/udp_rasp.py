import cv2
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2097152)

LAPTOP_IP = "192.168.1.12"
PORT = 9999

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]

while True:
    ret, frame = cap.read()
    if ret:
        _, buffer = cv2.imencode('.jpg', frame, encode_param)
        data = buffer.tobytes()
        
        if len(data) < 65000:
            sock.sendto(data, (LAPTOP_IP, PORT))

cap.release()
