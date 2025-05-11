# import RPi.GPIO as GPIO
# import time
# from gpiozero import DistanceSensor

# TRIG_PIN = 4
# ECHO_PIN = 17

# def setup_gpio():
#     """Set up GPIO pins for the ultrasonic sensor."""
#     GPIO.setmode(GPIO.BCM)
#     GPIO.setwarnings(False)
#     GPIO.setup(TRIG_PIN, GPIO.OUT)
#     GPIO.setup(ECHO_PIN, GPIO.IN)
#     GPIO.output(TRIG_PIN, False)
#     time.sleep(2)  # Allow sensor to settle

# def measure_distance():
#     """Measure distance using the ultrasonic sensor."""
#     # Send 10-microsecond trigger pulse
#     GPIO.output(TRIG_PIN, True)
#     time.sleep(0.00001)  # 10 us
#     GPIO.output(TRIG_PIN, False)

#     # Wait for ECHO to go high
#     start_time_raw = GPIO.wait_for_edge(ECHO_PIN, GPIO.RISING, timeout=5000)
#     if start_time_raw is None:
#         print("Error: No rising edge detected.")
#         return None

#     # Wait for ECHO to go low
#     end_time_raw = GPIO.wait_for_edge(ECHO_PIN, GPIO.FALLING, timeout=500)
#     if end_time_raw is None:
#         print("Error: No falling edge detected.")
#         return None

#     # Calculate distance (speed of sound = 34300 cm/s)
#     time_elapsed = end_time_raw - start_time_raw
#     distance = (time_elapsed * 34300) / 2
#     return distance

# def main():
#     setup_gpio()
#     try:
#         while True:
#             distance = measure_distance()
#             if distance is not None and 2 < distance < 400:
#                 print(f"Distance: {distance:.2f} cm")
#             else:
#                 print("Out of range or error")
#             time.sleep(1)
#     except KeyboardInterrupt:
#         print("Exiting...")
#     finally:
#         GPIO.cleanup()

# if __name__ == "__main__":
#     main()

# # from gpiozero import DistanceSensor
# # ultrasonic = DistanceSensor(echo=17, trigger=4)
# # while True:
# #     print(ultrasonic.distance)

import RPi.GPIO as GPIO
import time

# Pin definitions
TRIG_PIN = 4  # GPIO pin for TRIG
ECHO_PIN = 17  # GPIO pin for ECHO

# Variables for distance measurement
start_time = 0
end_time = 0

def setup_gpio():
    """Set up GPIO pins for the ultrasonic sensor."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.output(TRIG_PIN, False)
    time.sleep(2)  # Let sensor settle

def echo_callback(channel):
    """Callback function for ECHO pin events."""
    global start_time, end_time
    if GPIO.input(ECHO_PIN):  # Rising edge
        start_time = time.time()
    else:  # Falling edge
        end_time = time.time()

def measure_distance():
    """Measure distance using the ultrasonic sensor."""
    global start_time, end_time
    # Send trigger pulse
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # 10 microsecond pulse
    GPIO.output(TRIG_PIN, False)
    
    # Reset times
    start_time = 0
    end_time = 0
    
    # Wait for ECHO to complete (timeout after 40 ms)
    timeout = time.time() + 0.04  # Max range ~6.8 meters
    while time.time() < timeout:
        if start_time and end_time:
            break
        time.sleep(0.001)  # Reduce CPU usage
    
    if start_time and end_time:
        time_elapsed = end_time - start_time
        distance = (time_elapsed * 34300) / 2  # Speed of sound: 343 m/s
        return distance
    else:
        print("Error: No echo detected within timeout")
        return None

def main():
    """Main loop to continuously measure and display distance."""
    setup_gpio()
    GPIO.add_event_detect(ECHO_PIN, GPIO.BOTH, callback=echo_callback)
    try:
        while True:
            distance = measure_distance()
            if distance is not None and 2 < distance < 400:
                print(f"Distance: {distance:.2f} cm")
            else:
                print("Out of range or error")
            time.sleep(1)  # Measure every second
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()