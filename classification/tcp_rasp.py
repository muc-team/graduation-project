import cv2
import zmq
import base64

context = zmq.Context()

socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

camera = cv2.VideoCapture(0)

camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Streaming started...")

while True:
    ret, frame = camera.read()
    if not ret:
        continue

    encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

    socket.send(buffer)