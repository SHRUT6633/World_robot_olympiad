import serial
import time

ser = serial.Serial("/dev/serial0", 115200, timeout=1)
time.sleep(0.1)  # Wait for ESP32 reset
ser.reset_input_buffer()  # Flush bootloader garbage

ser.write(b"ping\n")
response = ser.readline().strip().decode()
print(f"Sent: ping   Received: {response}")

assert response == "pong", f"Expected pong, got {response}"
print("UART loopback test: PASS")
ser.close()
