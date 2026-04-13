import cv2
import time
from ultralytics import YOLO

processing_enabled = True
video_port = 0

model = YOLO('../models/yolov8n.pt')
cap = cv2.VideoCapture(video_port)
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
            
    else:
        cv2.putText(display_frame, "STATUS: PAUSED", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        

    cv2.imshow("YOLOv8 Live Segmentation (Voice Controlled)", display_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print("Closing...")
cap.release()
cv2.destroyAllWindows()
