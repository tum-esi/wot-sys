import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
SWITCH_GPIO = 17
GPIO.setup(SWITCH_GPIO, GPIO.IN)

while True:
    print(GPIO.input(SWITCH_GPIO))
    time.sleep(0.1)