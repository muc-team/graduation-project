import cv2
import zmq
import time
import base64

context = zmq.Context()
socket = context.socket(zmq.PUB)

socket.setsockopt(zmq.SNDHWM, 1)
socket.setsockopt(zmq.LINGER, 0)

socket.bind("tcp://*:5555")

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FPS, 30)

print("🚀 ZeroMQ Turbo Stream Started (Q35)...")

while True:
    ret, frame = camera.read()
    if not ret:
        continue

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 35]
    encoded, buffer = cv2.imencode('.jpg', frame, encode_param)
    
    try:
        socket.send(buffer, zmq.NOBLOCK) 
    except zmq.Again:
        pass