# sender.py (على الراسبيري)
import cv2
import zmq
import base64

context = zmq.Context()
# بنفتح بورت لنشر الصور (PUB)
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")  # هنبعت على بورت 5555

camera = cv2.VideoCapture(0)  # تأكد إن الكاميرا رقم 0 أو غيرها

# تقليل جودة الكاميرا شوية عشان السرعة
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Streaming started...")

while True:
    ret, frame = camera.read()
    if not ret:
        continue

    # 1. ضغط الصورة لـ JPEG (عشان حجمها يصغر والشبكة تستحمل)
    # الـ 80 دي جودة الصورة (ممكن تقللها لـ 50 لو عايز سرعة أعلى)
    encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    
    # 2. إرسال الصورة
    socket.send(buffer)