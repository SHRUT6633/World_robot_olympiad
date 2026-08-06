# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/simple_main.py
# Rev:  v10.0  |  Status: NEW
# -----------------------------------------------------------------------------
# Standalone race entry point — port of the proven /home/shrut/WRO_Test/main.py
# onto this repository's drivers.
#
# Why this file exists:
#   The old standalone script used adafruit_blinka, which crashed on the Pi
#   with a circular import inside adafruit_blinka.microcontroller.generic_linux
#   (i2c <-> Adafruit_PureIO.smbus <-> busio). This repo's drivers use
#   smbus2 + gpiozero instead, so the circular import can never occur.
#
# What is preserved (nothing changed):
#   - Sensor wiring: front VL53L1X @ 0x30 (XSHUT GPIO22),
#     left VL53L0X @ 0x31 (XSHUT GPIO17), right VL53L0X @ 0x32 (XSHUT GPIO27),
#     MPU6050 @ 0x68.
#   - Timing constants from the working code: POWER_DELAY, ADDRESS_DELAY,
#     LOOP_DELAY — used exactly as before.
#   - "ESP32 Connected" handshake over UART /dev/serial0 @ 115200.
#
# Run on the Raspberry Pi:  python -m pi.simple_main
# =============================================================================

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from pi.system.logger import log
log.init()

from pi.sensors.tof.vl53l0x import VL53L0X
from pi.sensors.tof.vl53l1x import VL53L1X
from pi.sensors.tof import xshut_manager
from pi.sensors.imu.mpu6050 import MPU6050
from pi.comm.uart import UARTCommunicator

# -----------------------------------------------------------------------------
# Proven timing constants (unchanged from the working standalone code)
# -----------------------------------------------------------------------------
POWER_DELAY = 0.125      # s — settle time after powering a sensor via XSHUT
ADDRESS_DELAY = 0.0625   # s — settle time after an I2C address write
LOOP_DELAY = 0.10        # s — main loop period (10 Hz)

# -----------------------------------------------------------------------------
# Proven hardware wiring (unchanged from the working standalone code)
# -----------------------------------------------------------------------------
FRONT_ADDRESS = 0x30
FRONT_XSHUT = 22
LEFT_ADDRESS = 0x31
LEFT_XSHUT = 17
RIGHT_ADDRESS = 0x32
RIGHT_XSHUT = 27
IMU_ADDRESS = 0x68
I2C_BUS = 1

UART_PORT = "/dev/serial0"
UART_BAUD = 115200

# -----------------------------------------------------------------------------
# Drive parameters
# -----------------------------------------------------------------------------
SERVO_CENTER = 90
SERVO_MAX_ANGLE = 30            # max steering deflection from center, degrees
SPEED_FAST = 200                # motor PWM (0-255)
SPEED_SLOW = 120
SPEED_STOP = 0

FRONT_DANGER_MM = 300           # brake/turn when the front is closer than this
SIDE_WALL_TARGET_MM = 150       # wall-following target gap on left and right
SIDE_ESCAPE_MM = 250            # steer away when a side gap opens this wide


def connect_esp32():
    # Keep trying until the connection is established. Never give up and
    # never continue into the sensor phase without the ESP32 link.
    uart = UARTCommunicator(port=UART_PORT, baudrate=UART_BAUD)
    attempt = 0
    fix_shown = False
    while True:
        attempt += 1
        uart.init()
        if uart._serial is not None:
            log.info(f"ESP32 Connected (attempt {attempt})")
            return uart
        err = uart._last_error
        if err is not None and not fix_shown:
            fix_shown = True
            if "Permission denied" in str(err):
                log.warn("UART permission fix (run once on the Pi):")
                log.warn("  sudo usermod -a -G dialout $USER")
                log.warn("  sudo reboot")
                log.warn("Or run this script with:  sudo python3 -m pi.simple_main")
            else:
                log.warn(f"UART error: {err}")
        log.warn(f"ESP32 connection attempt {attempt} failed, retrying...")
        uart.close()
        time.sleep(POWER_DELAY)


def clamp_servo(angle):
    return max(SERVO_CENTER - SERVO_MAX_ANGLE,
               min(SERVO_CENTER + SERVO_MAX_ANGLE, angle))


def main():
    # Construct ALL sensors first: each registers its XSHUT pin with the
    # shared manager, so the address programming of one sensor can hold
    # every other sensor in hardware reset. Timing between the steps below
    # keeps the proven POWER_DELAY / ADDRESS_DELAY pacing.
    front = VL53L1X("VL53L1X_Front", bus=I2C_BUS, address=FRONT_ADDRESS,
                    xshut_pin=FRONT_XSHUT)
    left = VL53L0X("VL53L0X_Left", bus=I2C_BUS, address=LEFT_ADDRESS,
                   xshut_pin=LEFT_XSHUT)
    right = VL53L0X("VL53L0X_Right", bus=I2C_BUS, address=RIGHT_ADDRESS,
                    xshut_pin=RIGHT_XSHUT)
    imu = MPU6050(bus=I2C_BUS, address=IMU_ADDRESS, accel_range=4,
                  gyro_range=500)
    uart = connect_esp32()

    # ---- ToF init: FIRST all OFF, THEN one by one ----
    # Every sensor starts in hardware reset so no address write is ever
    # seen by a sensor that is not being programmed.
    log.info("ToF init: turning ALL sensors OFF (XSHUT low)")
    held_all = xshut_manager.set_all_off()
    time.sleep(POWER_DELAY)

    # Front: power ON -> give address 0x30 -> start measuring -> take data
    xshut_manager.close_all(held_all)
    front.init()
    time.sleep(POWER_DELAY)
    log.info(f"Front first read: {front.read()} mm")

    # Left: power ON -> give address 0x31 -> start measuring -> take data
    left.init()
    time.sleep(POWER_DELAY)
    log.info(f"Left first read: {left.read()} mm")

    # Right: power ON -> give address 0x32 -> start measuring -> take data
    right.init()
    time.sleep(POWER_DELAY)
    log.info(f"Right first read: {right.read()} mm")

    imu.init()

    log.info("=" * 50)
    log.info("SIMPLE RACE MODE - READY")
    log.info("=" * 50)

    try:
        while True:
            d_front = front.read()
            d_left = left.read()
            d_right = right.read()
            imu_data = imu.read()
            yaw_rate = imu_data["gyro"][2] if imu_data else 0.0

            d_front = d_front if d_front is not None else 9999
            d_left = d_left if d_left is not None else 9999
            d_right = d_right if d_right is not None else 9999

            # Obstacle ahead: slow down and steer toward the open side.
            if d_front < FRONT_DANGER_MM:
                if d_left > d_right:
                    servo = SERVO_CENTER - SERVO_MAX_ANGLE  # turn left
                else:
                    servo = SERVO_CENTER + SERVO_MAX_ANGLE  # turn right
                speed = SPEED_SLOW
            else:
                # Wall-following: keep the nearest side gap at the target
                # distance; if both are open, hold centre.
                if d_left < SIDE_WALL_TARGET_MM:
                    servo = SERVO_CENTER + SERVO_MAX_ANGLE * 0.6  # away from wall
                elif d_right < SIDE_WALL_TARGET_MM:
                    servo = SERVO_CENTER - SERVO_MAX_ANGLE * 0.6
                elif d_left > SIDE_ESCAPE_MM or d_right > SIDE_ESCAPE_MM:
                    servo = SERVO_CENTER  # open space, keep straight
                else:
                    servo = SERVO_CENTER
                speed = SPEED_FAST

            servo = clamp_servo(servo)

            uart.send_steering(servo, speed)

            log.info(f"front={d_front:6.0f}mm left={d_left:6.0f}mm "
                     f"right={d_right:6.0f}mm yaw={yaw_rate:7.2f} "
                     f"-> servo={servo:5.1f} speed={speed}")
            time.sleep(LOOP_DELAY)
    except KeyboardInterrupt:
        log.info("Stopping...")
        uart.send_emergency_stop()
    finally:
        front.close()
        left.close()
        right.close()
        imu.close()
        uart.close()


if __name__ == "__main__":
    main()
