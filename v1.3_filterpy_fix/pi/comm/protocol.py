import struct
from enum import IntEnum


class PacketType(IntEnum):
    MOTOR_COMMAND = 0x01
    SERVO_COMMAND = 0x02
    STEERING_COMMAND = 0x03
    STATUS_REQUEST = 0x04
    STATUS_RESPONSE = 0x05
    EMERGENCY_STOP = 0xFF


class Packet:
    HEADER = 0xA5
    FOOTER = 0x5A

    def __init__(self):
        self.header = self.HEADER
        self.counter = 0
        self.msg_type = 0
        self.length = 0
        self.payload = b""
        self.crc = 0
        self.footer = self.FOOTER

    def encode(self):
        data = struct.pack("<BB", self.HEADER, self.counter)
        data += struct.pack("<B", self.msg_type)
        data += struct.pack("<B", len(self.payload))
        data += self.payload
        self.crc = self._crc16(data, 0x8005)
        data += struct.pack("<H", self.crc)
        data += struct.pack("<B", self.FOOTER)
        return data

    def decode(self, data):
        if isinstance(data, list):
            data = bytes(data)
        if len(data) < 8:
            return False
        if data[0] != self.HEADER or data[-1] != self.FOOTER:
            return False
        self.counter = data[1]
        self.msg_type = data[2]
        self.length = data[3]
        payload_end = 4 + self.length
        self.payload = data[4:payload_end]
        self.crc = struct.unpack("<H", data[payload_end:payload_end+2])[0]
        calculated = self._crc16(list(data[:payload_end]), 0x8005)
        return self.crc == calculated

    @staticmethod
    def _crc16(data, poly=0x8005):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    @staticmethod
    def make_steering_command(counter, servo_angle, motor_speed):
        pkt = Packet()
        pkt.counter = counter
        pkt.msg_type = PacketType.STEERING_COMMAND
        pkt.payload = struct.pack("<fB", servo_angle, int(motor_speed))
        return pkt

    @staticmethod
    def make_emergency_stop(counter):
        pkt = Packet()
        pkt.counter = counter
        pkt.msg_type = PacketType.EMERGENCY_STOP
        pkt.length = 0
        return pkt
