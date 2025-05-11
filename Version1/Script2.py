import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import RPi.GPIO as GPIO
import speech_recognition as sr
import time
import subprocess
import threading
import sys

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TRIG = 18
ECHO = 24
BUTTON_PIN = 23
THRESHOLD_DISTANCE = 50  # cm
AUDIO_COOLDOWN = 5  # Seconds between audio announcements
FRAME_SKIP = 2  # Process every 2nd frame

# Global variables for threading
latest_frame = None
detected_objects = []
lock = threading.Lock()
last_spoken_time = 0

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
    """Set up GPIO for ultrasonic sensor and button."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)  # Disable GPIO warnings
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def measure_distance():
    """Measure distance using ultrasonic sensor."""
    distances = []
    for _ in range(5):
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        start_time = time.time()
        end_time = time.time()
        while GPIO.input(ECHO) == 0:
            start_time = time.time()
        while GPIO.input(ECHO) == 1:
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
    """Continuously measure distance and alert for obstacles."""
    global last_spoken_time
    while True:
        distance = measure_distance()
        if distance and distance < THRESHOLD_DISTANCE and (time.time() - last_spoken_time) > AUDIO_COOLDOWN:
            speak("Obstacle ahead")
            last_spoken_time = time.time()
        time.sleep(0.1)

def main():
    """Main function to orchestrate the blind stick system."""
    # Setup hardware and model
    cap = setup_camera()
    setup_gpio()
    interpreter, input_details, output_details = load_model()
    speak("System ready")

    # Start threads
    capture_thread = threading.Thread(target=capture_frames, args=(cap,), daemon=True)
    distance_thread_obj = threading.Thread(target=distance_thread, daemon=True)
    capture_thread.start()
    distance_thread_obj.start()

    # Speech recognition setup
    r = sr.Recognizer()

    try:
        while True:
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                with sr.Microphone() as source:
                    speak("Listening")
                    audio = r.listen(source, timeout=3, phrase_time_limit=3)
                try:
                    command = r.recognize_pocketsphinx(audio)
                    if "describe the scene" in command.lower() or "what's in front of me" in command.lower():
                        with lock:
                            if latest_frame is not None:
                                detected_objects = detect_objects(latest_frame, interpreter, input_details, output_details)
                                if detected_objects:
                                    speak_text = "Detected: " + ", ".join(detected_objects[:3])  # Limit to 3 for brevity
                                    speak(speak_text)
                                else:
                                    speak("No objects detected")
                    else:
                        speak("Unknown command")
                except sr.UnknownValueError:
                    speak("Did not understand")
                time.sleep(AUDIO_COOLDOWN)  # Cooldown after command
            time.sleep(0.1)  # Small delay to reduce CPU usage
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("Resources released")

if __name__ == "__main__":
    main()