# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/comm/uart.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# UART serial communicator
# =============================================================================

import time
import threading
import struct
from collections import deque
from ..system.logger import log
from .protocol import Packet, PacketType


class UARTCommunicator:
    def __init__(self, port="/dev/serial0", baudrate=115200, timeout_ms=50):
        self.port = port          # Serial port device path (e.g., /dev/serial0 on Raspberry Pi)
        self.baudrate = baudrate  # Communication speed in baud
        self.timeout = timeout_ms / 1000.0  # Read/write timeout in seconds
        self._serial = None       # pyserial Serial object (None until init() succeeds)
        self._running = False     # Whether the communicator is active
        self._tx_counter = 0      # Outgoing packet counter (increments per send, wraps at 256)
        self._rx_counter = 0      # Incoming packet counter
        self._lock = threading.Lock()  # Protects serial writes from concurrent access
        # Ring buffer of recently received packets (max 100), allows decoupling read/process
        self._rx_buffer = deque(maxlen=100)
        # Timestamp (time.perf_counter) of last STATUS_RESPONSE heartbeat from ESP
        # Used to detect connection loss
        self._last_esp_heartbeat = 0.0

    def init(self):
        # Attempt to open the serial port. Logs success/failure but does NOT crash on failure,
        # allowing the rest of the system to operate in a mock/demo mode without physical hardware.
        try:
            import serial
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
            log.info(f"UART: {self.port} @ {self.baudrate} baud")
        except Exception as e:
            log.warn(f"UART init failed: {e}")

    def send(self, pkt: Packet):
        # Encode and write a single packet to the serial port.
        # Returns True on success, False if serial is unavailable or write fails.
        if self._serial is None:
            return False
        with self._lock:
            try:
                data = pkt.encode()
                self._serial.write(data)
                self._tx_counter += 1
                return True
            except Exception as e:
                log.warn(f"UART send error: {e}")
                return False

    def send_steering(self, servo_angle, motor_speed):
        # Convenience: builds a STEERING_COMMAND packet with the given servo angle (degrees)
        # and motor speed (0–255), then sends it. Counter auto-increments modulo 256.
        self._tx_counter = (self._tx_counter + 1) & 0xFF
        pkt = Packet.make_steering_command(self._tx_counter, servo_angle, motor_speed)
        return self.send(pkt)

    def send_emergency_stop(self):
        # Sends an EMERGENCY_STOP packet. Counter is NOT incremented (reuses current counter).
        pkt = Packet.make_emergency_stop(self._tx_counter)
        return self.send(pkt)

    def read(self):
        # Non-blocking read: checks if at least 8 bytes (minimum packet size) are available.
        # If a valid packet is decoded, it is appended to _rx_buffer and returned.
        # Returns None if no valid packet is available.
        if self._serial is None:
            return None
        try:
            if self._serial.in_waiting >= 8:
                data = self._serial.read(self._serial.in_waiting)
                pkt = Packet()
                if pkt.decode(list(data)):
                    self._rx_counter += 1
                    # Track heartbeats: STATUS_RESPONSE packets indicate ESP is alive
                    if pkt.msg_type == PacketType.STATUS_RESPONSE:
                        self._last_esp_heartbeat = time.perf_counter()
                    self._rx_buffer.append(pkt)
                    return pkt
        except Exception as e:
            log.warn(f"UART read error: {e}")
        return None

    @property
    def is_connected(self):
        # Returns True if a STATUS_RESPONSE was received within the last 1 second.
        # If heartbeat stops for >1s, the connection is considered lost.
        now = time.perf_counter()
        return (now - self._last_esp_heartbeat) < 1.0

    def close(self):
        # Gracefully shut down the serial connection.
        self._running = False
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
