import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import time
import subprocess
import sys
from picamera2 import Picamera2

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
AUDIO_COOLDOWN = 5  # Seconds between audio announcements
FRAME_SKIP = 2  # Process every 2nd frame

# Global variables
last_spoken_time = 0

# COCO class names (partial list, add more as needed)
class_names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
               'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
               'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
               'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
               'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
               'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'fan',
               'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
               'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
               'hair drier', 'toothbrush']

def setup_camera():
    """Initialize and configure the Raspberry Pi camera."""
    # cap = cv2.VideoCapture(0)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    # if not cap.isOpened():
    #     print("Error: Could not open camera.")
    #     sys.exit(1)
    # return cap
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (FRAME_WIDTH, FRAME_HEIGHT)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()
    return picam2



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

def main():
    """Main function to read the environment and provide audio feedback."""
    
    last_spoken_time = 0
    # Setup hardware and model
    cap = setup_camera()
    interpreter, input_details, output_details = load_model()
    speak("System ready")

    # Frame counter for skipping
    frame_count = 0

    try:
        while True:
            frame = cap.capture_array()
            # ret, frame = cap.read()
            # if not ret:
            #     print("Error: Failed to capture frame.")
            #     continue

            # Process every FRAME_SKIP frame
            if frame_count % FRAME_SKIP == 0:
                detected_objects = detect_objects(frame, interpreter, input_details, output_details)
                current_time = time.time()

                # Provide audio feedback if cooldown has elapsed
                if (current_time - last_spoken_time) > AUDIO_COOLDOWN:
                    if detected_objects:
                        speak_text = "Detected: " + ", ".join(detected_objects[:3])  # Limit to 3 for brevity
                        speak(speak_text)
                    else:
                        speak("No objects detected")
                    last_spoken_time = current_time

            frame_count += 1
            time.sleep(0.01)  # Small delay to prevent CPU overload

            # Optional: Display the frame (for debugging)
            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) == ord("q"):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # cap.release()
        # cv2.destroyAllWindows()  # Uncomment if using cv2.imshow
        print("Resources released")

if __name__ == "__main__":
    main()