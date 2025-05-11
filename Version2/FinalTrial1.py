import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import os
import time
from gpiozero import DistanceSensor, DigitalOutputDevice
import threading

# Pin assignments (example)
right_sensor = DistanceSensor(echo=4, trigger=17)
left_sensor = DistanceSensor(echo=27, trigger=22)
front_sensor = DistanceSensor(echo=10, trigger=9)
vibration_motor = DigitalOutputDevice(18)

# Vibration patterns
current_pattern = None  # 'front', 'left', 'right', or None

def vibration_thread_func():
    global current_pattern
    while True:
        if current_pattern == 'front':
            vibration_motor.on()
            time.sleep(0.5)
            vibration_motor.off()
            time.sleep(0.5)
        elif current_pattern == 'left':
            vibration_motor.on()
            time.sleep(0.2)
            vibration_motor.off()
            time.sleep(0.2)
        elif current_pattern == 'right':
            vibration_motor.on()
            time.sleep(0.8)
            vibration_motor.off()
            time.sleep(0.2)
        else:
            vibration_motor.off()
            time.sleep(0.1)

vibration_thread = threading.Thread(target=vibration_thread_func)
vibration_thread.start()

# Initialize Picamera2 with reduced size for better performance
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Load YOLO11n model
model = YOLO("yolo11n.pt")

# Speech settings
speech_interval = 5  # Speak every 5 seconds
last_speech_time = 0

# Frame dimensions
frame_width = 640
front_range = (frame_width // 3, 2 * frame_width // 3)
left_range = (0, frame_width // 3)
right_range = (2 * frame_width // 3, frame_width)

def get_object_position(box):
    center_x = (box.xyxy[0][0] + box.xyxy[0][2]) / 2
    if left_range[0] <= center_x < left_range[1]:
        return 'left'
    elif front_range[0] <= center_x < front_range[1]:
        return 'front'
    elif right_range[0] <= center_x < right_range[1]:
        return 'right'
    else:
        return 'unknown'

threshold_distance = 1  # in meters

while True:
    # Capture frame
    frame = picam2.capture_array()
    
    # Run YOLO inference
    results = model(frame)
    
    # Get distances from ultrasonic sensors
    front_dist = front_sensor.distance
    left_dist = left_sensor.distance
    right_dist = right_sensor.distance
    
    # Determine vibration pattern
    if front_dist < threshold_distance:
        current_pattern = 'front'
    elif left_dist < threshold_distance:
        current_pattern = 'left'
    elif right_dist < threshold_distance:
        current_pattern = 'right'
    else:
        current_pattern = None
    
    # Categorize detected objects
    front_objects = []
    for box in results[0].boxes:
        position = get_object_position(box)
        if position == 'front':
            front_objects.append(model.names[int(box.cls)])
    
    # Prepare TTS message
    if front_objects:
        object_type = front_objects[0]  # Take the first one
        distance = front_dist
        speech_text = f"Object: {object_type}, Distance: {distance:.2f} meters"
    else:
        speech_text = "No object ahead"
    
    # Check for speech
    current_time = time.time()
    if current_time - last_speech_time > speech_interval:
        os.system(f'espeak "{speech_text}"')
        last_speech_time = current_time
    
    # Annotate frame (optional for testing)
    # annotated_frame = results[0].plot()
    
    # Display frame (optional for testing)
    # cv2.imshow("Camera", annotated_frame)
    
    # Exit on 'q'
    if cv2.waitKey(1) == ord("q"):
        break

# Cleanup
cv2.destroyAllWindows()
picam2.stop()
vibration_motor.off()