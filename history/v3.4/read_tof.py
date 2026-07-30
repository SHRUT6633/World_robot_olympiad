import smbus2
import time
import VL53L0X  # Adafruit VL53L0X library
import VL53L1X  # ST VL53L1X library
import RPi.GPIO as GPIO

LEFT_XSHUT = 17
RIGHT_XSHUT = 27
FRONT_XSHUT = 22
MAX_RANGE_L0X = 2000
MAX_RANGE_L1X = 4000

GPIO.setmode(GPIO.BCM)
for pin in [LEFT_XSHUT, RIGHT_XSHUT, FRONT_XSHUT]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

time.sleep(0.1)
GPIO.output(LEFT_XSHUT, GPIO.HIGH)
time.sleep(0.01)
left = VL53L0X.VL53L0X(address=0x29)
left.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
left.set_device_address(0x30)

GPIO.output(RIGHT_XSHUT, GPIO.HIGH)
time.sleep(0.01)
right = VL53L0X.VL53L0X(address=0x29)
right.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
right.set_device_address(0x31)

GPIO.output(FRONT_XSHUT, GPIO.HIGH)
time.sleep(0.01)
front = VL53L1X.VL53L1X(address=0x29)
front.open()
front.start_ranging(2)  # 2 = LONG range mode

last_valid = {"left": 500, "right": 500, "front": 1000}

def read_left():
    d = left.get_distance()
    return d if 0 < d <= MAX_RANGE_L0X else last_valid["left"]

def read_right():
    d = right.get_distance()
    return d if 0 < d <= MAX_RANGE_L0X else last_valid["right"]

def read_front():
    d = front.get_distance()
    status = front.get_range_status()
    if d <= 0 or d > MAX_RANGE_L1X or status != 0:
        return last_valid["front"]
    return d

def read_all():
    l = read_left()
    r = read_right()
    f = read_front()
    last_valid["left"] = l
    last_valid["right"] = r
    last_valid["front"] = f
    return l, r, f

if __name__ == "__main__":
    for i in range(50):
        l, r, f = read_all()
        print(f"L={l:4d}  R={r:4d}  F={f:4d}")
        time.sleep(0.033)

    left.stop_ranging()
    right.stop_ranging()
    front.stop_ranging()
    GPIO.cleanup()
