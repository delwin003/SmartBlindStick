import RPi.GPIO as GPIO
import time

# GPIO Pin assignments
LIGHT_PIN_1 = 18
LIGHT_PIN_2 = 24

def setup_gpio():
    """Sets up GPIO pins."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LIGHT_PIN_1, GPIO.OUT)
    GPIO.setup(LIGHT_PIN_2, GPIO.OUT)
    GPIO.output(LIGHT_PIN_1, GPIO.LOW)  # Ensure lights are initially off
    GPIO.output(LIGHT_PIN_2, GPIO.LOW)

def rhythm_lights(pattern):
    """Turns lights on and off in a specified rhythm."""
    for state in pattern:
        if state == 1:
            GPIO.output(LIGHT_PIN_1, GPIO.HIGH)
            GPIO.output(LIGHT_PIN_2, GPIO.LOW)
        elif state == 2:
            GPIO.output(LIGHT_PIN_1, GPIO.LOW)
            GPIO.output(LIGHT_PIN_2, GPIO.HIGH)
        elif state == 3:
            GPIO.output(LIGHT_PIN_1, GPIO.HIGH)
            GPIO.output(LIGHT_PIN_2, GPIO.HIGH)
        else:
            GPIO.output(LIGHT_PIN_1, GPIO.LOW)
            GPIO.output(LIGHT_PIN_2, GPIO.LOW)

        time.sleep(0.5)  # Adjust timing as needed

def main():
    """Main function to run the rhythm lights."""
    setup_gpio()

    try:
        # Define the rhythm pattern (1: light1, 2: light2, 3: both, 0: off)
        rhythm = [1, 0, 2, 0, 3, 0, 1, 2, 0, 3, 0, 0] # example pattern
        while True:
            rhythm_lights(rhythm)
            time.sleep(1) # wait between rhythm loops.

    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()