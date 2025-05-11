# # from ultralytics import YOLO
# # import cv2
# # import pyttsx3
# # import threading
# # import time

# # # Initialize the TTS engine
# # engine = pyttsx3.init()
# # engine.setProperty('rate', 150)  # Speech speed (words per minute)
# # engine.setProperty('volume', 1)  # Volume (0 to 1)

# # # Load YOLOv8 Nano model
# # model = YOLO("yolov8n.pt")  # Lightweight Nano version

# # # Open the camera (0 for default camera; adjust if using USB camera)
# # cap = cv2.VideoCapture(0)
# # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)   # Lower resolution for speed
# # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# # # Function to announce detected objects
# # def speak_objects(objects):
# #     if objects:
# #         # Only announce unique objects to avoid repetition
# #         unique_objects = list(set(objects))
# #         announcement = "Detected: " + ", ".join(unique_objects)
# #         engine.say(announcement)
# #         engine.runAndWait()

# # # Main loop variables
# # detected_objects = []
# # last_spoken_time = 0
# # speak_interval = 5  # Announce every 5 seconds or when new objects appear

# # # Start real-time detection
# # while True:
# #     ret, frame = cap.read()
# #     if not ret:
# #         print("Failed to capture frame. Check camera connection.")
# #         break

# #     # Run YOLOv8 Nano object detection
# #     results = model(frame, conf=0.5)  # Confidence threshold of 0.5

# #     # Extract detected objects
# #     current_objects = []
# #     for result in results:
# #         for box in result.boxes:
# #             class_id = int(box.cls)
# #             class_name = result.names[class_id]
# #             current_objects.append(class_name)

# #     # Speak if new objects are detected or interval has passed
# #     current_time = time.time()
# #     if current_objects != detected_objects or (current_time - last_spoken_time) > speak_interval:
# #         detected_objects = current_objects
# #         if detected_objects:
# #             # Use a separate thread for TTS to avoid blocking
# #             threading.Thread(target=speak_objects, args=(detected_objects,)).start()
# #             last_spoken_time = current_time

# #     # Optional: Uncomment to display the video feed for debugging
# #     cv2.imshow("YOLOv8 Nano", frame)
# #     if cv2.waitKey(1) & 0xFF == ord('q'):
# #         break

# # # Clean up resources
# # cap.release()
# # cv2.destroyAllWindows()

# import cv2

# # Define the GStreamer pipeline for libcamera
# def gstreamer_pipeline(
#     capture_width=640,    # Resolution for capturing
#     capture_height=480,
#     display_width=640,    # Resolution for display
#     display_height=480,
#     framerate=30,         # Frames per second
#     flip_method=0,        # 0 = no flip, 2 = 180-degree rotation
# ):
#     return (
#         f"libcamerasrc ! "
#         f"video/x-raw, width={capture_width}, height={capture_height}, framerate={framerate}/1 ! "
#         f"videoflip method={flip_method} ! "
#         f"videoconvert ! "
#         f"appsink"
#     )

# # Initialize the camera with the GStreamer pipeline
# cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

# # Check if the camera opened successfully
# if not cap.isOpened():
#     print("Error: Cannot open camera")
#     exit()

# # Main loop to capture and display frames
# while True:
#     ret, frame = cap.read()  # Read a frame from the camera
#     if not ret:
#         print("Error: Can't receive frame. Exiting...")
#         break

#     # Optional: Add your frame processing here (e.g., object detection)
#     # Example: frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

#     # Display the frame in a window
#     cv2.imshow('Camera Feed', frame)

#     # Exit the loop by pressing 'q'
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Clean up: release the camera and close windows
# cap.release()
# cv2.destroyAllWindows()

# def gstreamer_pipeline(
#     capture_width=640,    # Resolution for capturing
#     capture_height=480,
#     display_width=640,    # Resolution for display
#     display_height=480,
#     framerate=30,         # Frames per second
#     flip_method=0,        # 0 = no flip, 2 = 180-degree rotation
# ):
#     return (
#         f"libcamerasrc ! "
#         f"video/x-raw, width={capture_width}, height={capture_height}, framerate={framerate}/1 ! "
#         f"videoflip method={flip_method} ! "
#         f"videoconvert ! "
#         f"appsink"
#     )

# import cv2
# print(cv2.getBuildInformation())

import cv2

cap = cv2.VideoCapture("http://localhost:8090")
if not cap.isOpened():
    print("Error: Cannot open camera stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame. Exiting...")
        break
    cv2.imshow('Camera Feed', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()