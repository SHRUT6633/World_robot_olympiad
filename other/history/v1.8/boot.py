import sys
sys.path.insert(0, ".")

print("=== WRO 2026 Self-Test ===")
all_pass = True

# Test 1: GPIO LEDs
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(23, GPIO.OUT)
    GPIO.output(23, 1)
    print("[PASS] Green LED")
    GPIO.cleanup()
except Exception as e:
    print(f"[FAIL] LED: {e}")
    all_pass = False

# Test 2: I2C bus
try:
    import smbus2
    bus = smbus2.SMBus(1)
    bus.read_byte(0x68)  # MPU6050
    print("[PASS] I2C bus + MPU6050")
except Exception as e:
    print(f"[FAIL] I2C: {e}")
    all_pass = False

# Test 3: UART
try:
    import serial
    ser = serial.Serial("/dev/serial0", 115200, timeout=1)
    ser.close()
    print("[PASS] UART port")
except Exception as e:
    print(f"[FAIL] UART: {e}")
    all_pass = False

print(f"\nSelf-test: {'ALL PASS' if all_pass else 'FAILED'}")
sys.exit(0 if all_pass else 1)
