import RPi.GPIO as GPIO

PIN = 3

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)
GPIO.output(PIN, 0)
# GPIO.output(PIN, 1)