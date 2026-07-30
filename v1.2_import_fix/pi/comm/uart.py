import time
import threading
import struct
from collections import deque
from ..system.logger import log
from .protocol import Packet, PacketType


class UARTCommunicator:
    def __init__(self, port="/dev/serial0", baudrate=115200, timeout_ms=50):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout_ms / 1000.0
        self._serial = None
        self._running = False
        self._tx_counter = 0
        self._rx_counter = 0
        self._lock = threading.Lock()
        self._rx_buffer = deque(maxlen=100)
        self._last_esp_heartbeat = 0.0

    def init(self):
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
        self._tx_counter = (self._tx_counter + 1) & 0xFF
        pkt = Packet.make_steering_command(self._tx_counter, servo_angle, motor_speed)
        return self.send(pkt)

    def send_emergency_stop(self):
        pkt = Packet.make_emergency_stop(self._tx_counter)
        return self.send(pkt)

    def read(self):
        if self._serial is None:
            return None
        try:
            if self._serial.in_waiting >= 8:
                data = self._serial.read(self._serial.in_waiting)
                pkt = Packet()
                if pkt.decode(list(data)):
                    self._rx_counter += 1
                    if pkt.msg_type == PacketType.STATUS_RESPONSE:
                        self._last_esp_heartbeat = time.perf_counter()
                    self._rx_buffer.append(pkt)
                    return pkt
        except Exception as e:
            log.warn(f"UART read error: {e}")
        return None

    @property
    def is_connected(self):
        now = time.perf_counter()
        return (now - self._last_esp_heartbeat) < 1.0

    def close(self):
        self._running = False
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
