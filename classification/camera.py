import cv2
import time
import pyttsx3 
import threading
from ultralytics import YOLO

processing_enabled = True
last_announced_object = None
video_port = 0
engine = pyttsx3.init()
engine.setProperty('rate', 150)

def speak(text):
    print(f"!!! Voice Alert: {text} !!!")
    engine.say(text)
    engine.runAndWait()

model = YOLO('../models/yolov8n.pt')

cap = cv2.VideoCapture(video_port)
print("Camera is on... Press 'q' to quit")
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    display_frame = frame.copy()

    current_object_found = None

    if processing_enabled:
        results = model(frame)
        display_frame = results[0].plot() 
        cv2.putText(display_frame, "STATUS: PROCESSING", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        if results[0].boxes: 
            first_object_cls_id = int(results[0].boxes.cls[0])
            current_object_found = model.names[first_object_cls_id]

        if current_object_found != last_announced_object:
            
            if current_object_found is not None:
                speech_thread = threading.Thread(
                    target=speak, 
                    args=(f"I found a {current_object_found}",),
                    daemon=True
                )
                speech_thread.start()
            
            last_announced_object = current_object_found
            
    else:
        cv2.putText(display_frame, "STATUS: PAUSED", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        last_announced_object = None

    cv2.imshow("YOLOv8 Live Segmentation (Voice Controlled)", display_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print("Closing...")
cap.release()
cv2.destroyAllWindows()
