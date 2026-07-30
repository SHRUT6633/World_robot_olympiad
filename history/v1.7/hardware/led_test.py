import RPi.GPIO as GPIO
import time

GREEN_LED = 23
RED_LED = 24
SWITCH = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(GREEN_LED, GPIO.OUT)
GPIO.setup(RED_LED, GPIO.OUT)
GPIO.setup(SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def debounce_read(pin, samples=5, interval=0.01):
    results = [GPIO.input(pin) for _ in range(samples)]
    return 1 if sum(results) > samples // 2 else 0

print("LED Test: blinking green")
for _ in range(5):
    GPIO.output(GREEN_LED, 1)
    time.sleep(0.5)
    GPIO.output(GREEN_LED, 0)
    time.sleep(0.5)

print("Press the start switch...")
while debounce_read(SWITCH) == 1:  # Pulled up, press = low
    time.sleep(0.01)
print("Switch detected!")

GPIO.cleanup()
