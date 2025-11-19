import cv2
import time
import pyttsx3 
import threading
from ultralytics import YOLO
import speech_recognition as sr

processing_enabled = False
last_announced_object = None


def voice_listener():
    global processing_enabled
    r = sr.Recognizer()

    print("Voice listener started... Say 'go' or 'stop'")

    while True:
        try:
            with sr.Microphone() as source:
                r.adjust_for_ambient_noise(source, duration=0.5)
                audio = r.listen(source)

            command = r.recognize_google(audio, language="en-US").lower()
            print(f"Command heard: {command}")

            if "go" in command:
                processing_enabled = True
                print("--- Status: Classification enabled ---")
            elif "stop" in command:
                processing_enabled = False
                print("--- Status: Classification paused ---")

        except sr.UnknownValueError:
            pass
        except sr.RequestError as e:
            print(f"Google Speech Recognition service error; {e}")
        except Exception as e:
            print(f"Error in voice listener thread: {e}")


engine = pyttsx3.init()
engine.setProperty('rate', 150)

def speak(text):
    print(f"!!! Voice Alert: {text} !!!")
    engine.say(text)
    engine.runAndWait()

model = YOLO('../models/yolov8n-seg.pt')

cap = cv2.VideoCapture(0)
print("Camera is on... Press 'q' to quit")

listener_thread = threading.Thread(target=voice_listener, daemon=True)
listener_thread.start()


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
