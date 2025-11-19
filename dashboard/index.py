import cv2
import time
import queue
import pyttsx3 
import threading
import numpy as np
import streamlit as st
from ultralytics import YOLO
import speech_recognition as sr

st.set_page_config(page_title="Rescue Robot", page_icon="🤖", layout="wide")

@st.cache_resource
def load_model():
    return YOLO('../models/yolov8n-seg.pt')

if 'speech_queue' not in st.session_state:
    st.session_state['speech_queue'] = queue.Queue()

def speak_worker(text_queue):
    engine = pyttsx3.init()
    engine.setProperty('rate', 150)
    while True:
        try:
            text = text_queue.get(timeout=1)
            if text == "CLEAR_QUEUE":
                with text_queue.mutex:
                    text_queue.queue.clear()
                continue
                
            engine.say(text)
            engine.runAndWait()
        except queue.Empty:
            pass
        except Exception as e:
            pass

def voice_worker():
    r = sr.Recognizer()
    while True:
        try:
            with sr.Microphone() as source:
                r.adjust_for_ambient_noise(source, duration=0.5)
                audio = r.listen(source, phrase_time_limit=2)
            
            command = r.recognize_google(audio, language="en-US").lower()
            
            if "go" in command:
                st.session_state['processing_enabled'] = True
            elif "stop" in command:
                st.session_state['processing_enabled'] = False
                while not st.session_state['speech_queue'].empty():
                    st.session_state['speech_queue'].get()
                
        except:
            pass

if 'threads_started' not in st.session_state:
    t_tts = threading.Thread(target=speak_worker, args=(st.session_state['speech_queue'],), daemon=True)
    t_tts.start()
    
    t_voice = threading.Thread(target=voice_worker, daemon=True)
    t_voice.start()
    
    st.session_state['threads_started'] = True

st.sidebar.title("⚙️ Control Panel")
confidence = st.sidebar.slider("Detection Confidence", 0.1, 1.0, 0.4)
enable_voice = st.sidebar.checkbox("Voice Announcements", value=True)

st.title("🤖 Rescue Robot Dashboard")

col1, col2 = st.columns([2, 1])
with col1:
    st.subheader("Live Feed")
    video_placeholder = st.empty()
    
    if 'run_camera' not in st.session_state:
        st.session_state['run_camera'] = False

    b1, b2 = st.columns(2)
    with b1:
        if st.button("▶️ Start", type="primary", use_container_width=True):
            st.session_state['run_camera'] = True
            st.session_state['processing_enabled'] = True
            st.rerun()

    with b2:
        if st.button("⏹️ Stop", use_container_width=True):
            st.session_state['run_camera'] = False
            st.session_state['processing_enabled'] = False
            st.session_state['speech_queue'].put("CLEAR_QUEUE")
            with st.session_state['speech_queue'].mutex:
                 st.session_state['speech_queue'].queue.clear()
            st.rerun()

with col2:
    st.subheader("Data")
    last_obj_metric = st.empty()

if st.session_state['run_camera']:
    cap = cv2.VideoCapture(0)
    model = load_model()
    
    last_announced = None
    
    while st.session_state['run_camera']:
        ret, frame = cap.read()
        if not ret:
            st.error("Camera error")
            break
            
        display_frame = frame.copy()
        
        if st.session_state.get('processing_enabled', False):            
            results = model(frame, conf=confidence, verbose=False)
            display_frame = results[0].plot()
            
            current_obj = None
            if results[0].boxes:
                cls_id = int(results[0].boxes.cls[0])
                current_obj = model.names[cls_id]
                
                if current_obj != last_announced:
                    if enable_voice:
                         st.session_state['speech_queue'].put(f"Found {current_obj}")
                    last_announced = current_obj
            
            last_obj_metric.metric("Detected", current_obj if current_obj else "Scanning...")
            
        else:
            cv2.putText(display_frame, "PAUSED", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            last_announced = None

        display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        video_placeholder.image(display_frame, channels="RGB", use_container_width=True)
        
        time.sleep(0.01) 
        
    cap.release()
else:
    video_placeholder.info("System Stopped.")
