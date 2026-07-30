print("WRO 2026 Robot Starting...")
print("System: Raspberry Pi 4 + ESP32-S3")
print("Status: Initializing hardware...")
# First attempt - no sys.path fix yet
from sensors.camera import PiCamera  # This will fail!
