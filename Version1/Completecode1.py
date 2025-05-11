import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import RPi.GPIO as GPIO
import time
import subprocess
import threading
import sys

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TRIG_PINS = [18, 17, 27]  # Front, Left, Right
ECHO_PINS = [24, 23, 22]
MOTOR_PIN = 12
THRESHOLD_DISTANCE = 50  # cm
AUDIO_COOLDOWN = 5  # Seconds between audio announcements
FRAME_SKIP = 2  # Process every 2nd frame
VIBRATION_THRESHOLD = 50  # cm
VIBRATION_PATTERN = {
    'front': (100, 0),  # Duty cycle, frequency (continuous for front)
    'left': (50, 10),   # Short bursts for left
    'right': (50, 5)    # Long bursts for right
}

# Global variables
latest_frame = None
distances = [0, 0, 0]  # Front, Left, Right
last_spoken_time = 0
lock = threading.Lock()

# COCO class names (partial list, add more as needed)
class_names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
               'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
               'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
               'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
               'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
               'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
               'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
               'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
               'hair drier', 'toothbrush']

def setup_camera():
    """Initialize and configure the Raspberry Pi camera."""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        sys.exit(1)
    return cap

def setup_gpio():
    """Set up GPIO for ultrasonic sensors and motor."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for trig in TRIG_PINS:
        GPIO.setup(trig, GPIO.OUT)
    for echo in ECHO_PINS:
        GPIO.setup(echo, GPIO.IN)
    GPIO.setup(MOTOR_PIN, GPIO.OUT)

def measure_distance(trig, echo):
    """Measure distance using ultrasonic sensor."""
    distances = []
    for _ in range(5):
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)
        start_time = time.time()
        end_time = time.time()
        while GPIO.input(echo) == 0:
            start_time = time.time()
        while GPIO.input(echo) == 1:
            end_time = time.time()
        time_elapsed = end_time - start_time
        distance = (time_elapsed * 34300) / 2  # cm
        if 2 < distance < 400:
            distances.append(distance)
    return sum(distances) / len(distances) if distances else None

def load_model():
    """Load the MobileNet V1 SSD model."""
    try:
        interpreter = tflite.Interpreter(model_path='coco_ssd_mobilenet_v1_1.0_quant.tflite')
        interpreter.allocate_tensors()
        return interpreter, interpreter.get_input_details(), interpreter.get_output_details()
    except Exception as e:
        print(f"Error loading model: {e}")
        sys.exit(1)

def detect_objects(frame, interpreter, input_details, output_details):
    """Run object detection on the frame."""
    resized_frame = cv2.resize(frame, (300, 300))
    input_data = np.expand_dims(resized_frame, axis=0).astype(np.uint8)
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    boxes = interpreter.get_tensor(output_details[0]['index'])[0]
    classes = interpreter.get_tensor(output_details[1]['index'])[0]
    scores = interpreter.get_tensor(output_details[2]['index'])[0]
    detected_objects = []
    for i in range(len(scores)):
        if scores[i] > 0.5:  # Confidence threshold
            class_id = int(classes[i])
            object_name = class_names[class_id]
            detected_objects.append(object_name)
    return detected_objects

def determine_section(center_x):
    """Determine if object is in left, center, or right section of frame."""
    section_width = FRAME_WIDTH // 3
    if center_x < section_width:
        return 'left', distances[1]  # Left sensor
    elif center_x < 2 * section_width:
        return 'center', distances[0]  # Front sensor
    else:
        return 'right', distances[2]  # Right sensor

def speak(text):
    """Use espeak to provide audio feedback."""
    try:
        subprocess.run(['espeak', text], check=True)
    except FileNotFoundError:
        print("Error: 'espeak' not found. Please install it with 'sudo apt install espeak'.")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"Error with espeak: {e}")

def capture_frames(cap):
    """Continuously capture frames from the camera."""
    global latest_frame
    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        with lock:
            if frame_count % FRAME_SKIP == 0:
                latest_frame = frame.copy()
        frame_count += 1
        time.sleep(0.01)  # Small delay to prevent CPU overload

def distance_thread():
    """Continuously measure distances from all three sensors."""
    global distances
    while True:
        for i in range(3):
            distance = measure_distance(TRIG_PINS[i], ECHO_PINS[i])
            with lock:
                distances[i] = distance if distance is not None else distances[i]
        time.sleep(0.1)

def control_motor():
    """Control vibration motor based on closest distance and direction."""
    pwm = None
    while True:
        with lock:
            min_distance = min(d for d in distances if d is not None)
            if min_distance < VIBRATION_THRESHOLD:
                # Determine direction with the smallest distance
                direction = 'front' if distances[0] == min_distance else \
                           'left' if distances[1] == min_distance else 'right'
                pattern = VIBRATION_PATTERN[direction]
                if pattern[1] == 0:  # Continuous vibration for front
                    if pwm is None:
                        pwm = GPIO.PWM(MOTOR_PIN, 100)  # 100 Hz for continuous
                    pwm.start(pattern[0])
                else:  # Bursts for left and right
                    if pwm is None:
                        pwm = GPIO.PWM(MOTOR_PIN, pattern[1])  # Frequency for bursts
                    pwm.start(pattern[0])
                    time.sleep(0.1)  # Short on time
                    pwm.stop()
                    time.sleep(0.1 if direction == 'left' else 0.5)  # Off time, short for left, long for right
            else:
                if pwm is not None:
                    pwm.stop()
                    pwm = None
        time.sleep(0.1)

def main():
    """Main function to read environment and provide feedback."""
    global last_spoken_time
    # Setup hardware and model
    cap = setup_camera()
    setup_gpio()
    interpreter, input_details, output_details = load_model()
    speak("System ready")

    # Start threads
    capture_thread = threading.Thread(target=capture_frames, args=(cap,), daemon=True)
    distance_thread_obj = threading.Thread(target=distance_thread, daemon=True)
    motor_thread = threading.Thread(target=control_motor, daemon=True)
    capture_thread.start()
    distance_thread_obj.start()
    motor_thread.start()

    # Continuous object detection and audio feedback
    frame_count = 0
    try:
        while True:
            with lock:
                if latest_frame is None:
                    continue
                frame = latest_frame.copy()

            if frame_count % FRAME_SKIP == 0:
                detected_objects = detect_objects(frame, interpreter, input_details, output_details)
                current_time = time.time()

                # Provide audio feedback if cooldown has elapsed
                if (current_time - last_spoken_time) > AUDIO_COOLDOWN and detected_objects:
                    for obj in detected_objects:
                        # Find the center of the frame for each detected object (simplified)
                        # This is a placeholder; you'd need to get bounding boxes for accurate positioning
                        center_x = FRAME_WIDTH // 2  # Placeholder, replace with actual object center
                        direction, distance = determine_section(center_x)
                        if distance < VIBRATION_THRESHOLD and distance is not None:
                            speak_text = f"{obj} {direction} at {int(distance)} cm"
                            speak(speak_text)
                    last_spoken_time = current_time
                elif (current_time - last_spoken_time) > AUDIO_COOLDOWN and not detected_objects:
                    speak("No objects detected")
                    last_spoken_time = current_time

            frame_count += 1
            time.sleep(0.01)  # Small delay to prevent CPU overload

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        cap.release()
        GPIO.cleanup()
        print("Resources released")

if __name__ == "__main__":
    main()