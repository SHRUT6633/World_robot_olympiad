import time
import serial
import board
import busio

from digitalio import DigitalInOut, Direction

import adafruit_vl53l1x
import adafruit_vl53l0x
from mpu6050 import mpu6050


# =====================
# DELAY
# =====================

POWER_DELAY = 0.125
ADDRESS_DELAY = 0.0625
LOOP_DELAY = 0.10

# =====================
# I2C ADDRESS
# =====================

FRONT_ADDR = 0x30
LEFT_ADDR  = 0x31
RIGHT_ADDR = 0x32
MPU_ADDR   = 0x68

# =====================
# XSHUT PINS
# =====================

FRONT_XSHUT = board.D22
LEFT_XSHUT  = board.D17
RIGHT_XSHUT = board.D27


# ==========================================

esp = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.05
)

time.sleep(2)

print("ESP32 Connected")


# ==========================================
# I2C
# ==========================================

i2c = busio.I2C(board.SCL, board.SDA)


# ==========================================
# XSHUT
# ==========================================

front_pin = DigitalInOut(FRONT_XSHUT)
left_pin  = DigitalInOut(LEFT_XSHUT)
right_pin = DigitalInOut(RIGHT_XSHUT)

for pin in (front_pin, left_pin, right_pin):
    pin.direction = Direction.OUTPUT
    pin.value = False

time.sleep(POWER_DELAY)


# ==========================================
# FRONT VL53L1X
# ==========================================

front_pin.value = True
time.sleep(POWER_DELAY)

front = adafruit_vl53l1x.VL53L1X(i2c)

front.start_ranging()

time.sleep(0.2)

front.set_address(FRONT_ADDR)

time.sleep(ADDRESS_DELAY)


# ==========================================
# LEFT VL53L0X
# ==========================================

left_pin.value = True
time.sleep(POWER_DELAY)

left = adafruit_vl53l0x.VL53L0X(i2c)

left.set_address(LEFT_ADDR)

time.sleep(ADDRESS_DELAY)


# ==========================================
# RIGHT VL53L0X
# ==========================================

right_pin.value = True
time.sleep(POWER_DELAY)

right = adafruit_vl53l0x.VL53L0X(i2c)

right.set_address(RIGHT_ADDR)

time.sleep(ADDRESS_DELAY)


print("Sensor Address Setup Complete")


# ==========================================
# MPU6050
# ==========================================

mpu = mpu6050(MPU_ADDR)

print("All Sensors Ready")


# ==========================================
# UART SEND
# ==========================================

def send_to_esp(servo, pwm, direction):

    packet = f"<{servo},{pwm},{direction}>"

    esp.write(packet.encode())

    if esp.in_waiting:
        print("ESP32 :", esp.readline().decode().strip())


# ==========================================
# MAIN LOOP
# ==========================================

print("\nRobot Started...\n")

while True:

    if front.data_ready:
        front_distance = front.distance
        front.clear_interrupt()
    else:
        front_distance = -1

    left_distance = left.range
    right_distance = right.range

    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()

    servo = 90
    motor = 150
    direction = 1

    if front_distance != -1:

        if front_distance < 150:
            motor = 0
            direction = 0

        else:

            if left_distance < right_distance:
                servo = 120

            elif right_distance < left_distance:
                servo = 60

            else:
                servo = 90

    send_to_esp(
        servo,
        motor,
        direction
    )

    print("===================================")
    print("Front :", front_distance, "mm")
    print("Left  :", left_distance, "mm")
    print("Right :", right_distance, "mm")

    print()

    print("ACC X :", round(accel["x"], 2))
    print("ACC Y :", round(accel["y"], 2))
    print("ACC Z :", round(accel["z"], 2))

    print()

    print("GYRO X :", round(gyro["x"], 2))
    print("GYRO Y :", round(gyro["y"], 2))
    print("GYRO Z :", round(gyro["z"], 2))

    print()

    print("Servo :", servo)
    print("Motor :", motor)
    print("Dir   :", direction)

    time.sleep(LOOP_DELAY)
