import cv2
import zmq
import time
import base64

context = zmq.Context()
socket = context.socket(zmq.PUB)

# --- إعدادات أقصى سرعة وإلغاء التخزين المؤقت للإرسال ---
# 1. SNDHWM=1: السماح بـ Buffer يحتوي رسالة واحدة فقط. أي رسالة إضافية يتم إسقاطها.
socket.setsockopt(zmq.SNDHWM, 1)
# 2. LINGER=0: عدم الانتظار عند الإغلاق.
socket.setsockopt(zmq.LINGER, 0)

socket.bind("tcp://*:5555")

camera = cv2.VideoCapture(0)
# دقة الصورة: 640x480 هي الحد الأدنى الذي يجب أن يعمل بشكل جيد
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FPS, 30)

print("🚀 ZeroMQ Turbo Stream Started (Q35)...")

while True:
    ret, frame = camera.read()
    if not ret:
        continue

    # 3. JPEG Quality 35: تقليل حجم البيانات بشكل كبير جداً
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 35]
    encoded, buffer = cv2.imencode('.jpg', frame, encode_param)
    
    try:
        # 4. zmq.NOBLOCK: الإرسال بدون انتظار، إذا كان المقبس مشغولاً، يتم إسقاط الإطار فوراً
        socket.send(buffer, zmq.NOBLOCK) 
    except zmq.Again:
        pass # إسقاط الإطار إذا كان قديماً
