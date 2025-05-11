import RPi.GPIO as GPIO
import time

def measure_distance(trig_pin, echo_pin):
    """Measure distance using ultrasonic sensor."""
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(trig_pin, False)

    start_time_raw = GPIO.wait_for_edge(echo_pin, GPIO.RISING, timeout=100) #timeout in milliseconds
    if start_time_raw is None:
        return None  # No response

    end_time_raw = GPIO.wait_for_edge(echo_pin, GPIO.FALLING, timeout=100) #timeout in milliseconds
    if end_time_raw is None:
        return None  # No response

    start_time = time.time() - (100/1000) + (start_time_raw/1000000)
    end_time = time.time() - (100/1000) + (end_time_raw/1000000)

    time_elapsed = end_time - start_time
    distance = (time_elapsed * 34300) / 2  # Speed of sound 34300 cm/s
    return distance

def setup_gpio():
    """Set up GPIO pins for ultrasonic sensors."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Set up TRIG pins as output and initially low
    trig_pins = [18, 17, 27]  # Front, Left, Right
    for trig in trig_pins:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)

    # Set up ECHO pins as input
    echo_pins = [24, 23, 22]  # Front, Left, Right
    for echo in echo_pins:
        GPIO.setup(echo, GPIO.IN)

if __name__ == "__main__":
    setup_gpio()

    try:
        while True:
            front_distance = measure_distance(18, 24)
            left_distance = measure_distance(17, 23)
            right_distance = measure_distance(27, 22)

            print(f"Front: {front_distance if front_distance else 'Out of range'} cm")
            print(f"Left: {left_distance if left_distance else 'Out of range'} cm")
            print(f"Right: {right_distance if right_distance else 'Out of range'} cm")
            print("--------------------------")

            time.sleep(1)  # Wait 1 second before next measurement
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        GPIO.cleanup()