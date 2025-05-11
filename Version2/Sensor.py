from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(echo=4, trigger=17)
while True:
    print('Distance: ', sensor.distance * 100)
    sleep(1)
