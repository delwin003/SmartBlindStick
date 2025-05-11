import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import os
import time

# Initialize Picamera2 with reduced size for better performance
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)  # Reduced from 1280x720
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Load YOLO11n model
model = YOLO("yolo11n.pt")

# Speech settings
speech_interval = 3  # Speak every 3 seconds
last_speech_time = 0

while True:
    # Capture frame
    frame = picam2.capture_array()
    
    # Run YOLO inference
    results = model(frame)
    
    # Annotate frame (optional for testing)
    annotated_frame = results[0].plot()
    
    # Check for speech
    current_time = time.time()
    if current_time - last_speech_time > speech_interval and results[0].boxes:
        detected_objects = list(set([model.names[int(box.cls)] for box in results[0].boxes]))
        speech_text = "Ahead: " + ", ".join(detected_objects)
        os.system(f'espeak "{speech_text}"')
        last_speech_time = current_time
    
    # Display frame (optional for testing)
    # cv2.imshow("Camera", annotated_frame)
    
    # Exit on 'q'
    if cv2.waitKey(1) == ord("q"):
        break

# Cleanup
cv2.destroyAllWindows()
picam2.stop()