import struct
from enum import IntEnum


class PacketType(IntEnum):
    # 0x01: Sent from Pi to ESP to set motor PWM speed
    MOTOR_COMMAND = 0x01
    # 0x02: Sent from Pi to ESP to set servo angle
    SERVO_COMMAND = 0x02
    # 0x03: Combined steering command (servo angle + motor speed) in one packet
    STEERING_COMMAND = 0x03
    # 0x04: Pi requests status data (battery, heartbeat, etc.) from ESP
    STATUS_REQUEST = 0x04
    # 0x05: ESP replies with status data
    STATUS_RESPONSE = 0x05
    # 0xFF: Emergency stop – immediately cuts motor power on ESP
    EMERGENCY_STOP = 0xFF


class Packet:
    # Fixed byte that marks the start of every packet
    HEADER = 0xA5
    # Fixed byte that marks the end of every packet
    FOOTER = 0x5A

    def __init__(self):
        self.header = self.HEADER
        self.counter = 0       # Transaction counter (0–255, wraps around)
        self.msg_type = 0      # PacketType value
        self.length = 0        # Number of bytes in payload
        self.payload = b""     # Variable-length data bytes
        self.crc = 0           # CRC-16 checksum over header+counter+type+length+payload
        self.footer = self.FOOTER

    def encode(self):
        # Pack header and counter as little-endian unsigned bytes
        data = struct.pack("<BB", self.HEADER, self.counter)
        # Append message type as unsigned byte
        data += struct.pack("<B", self.msg_type)
        # Append payload length as unsigned byte
        data += struct.pack("<B", len(self.payload))
        # Append the payload itself (raw bytes)
        data += self.payload
        # Compute CRC-16 over everything so far (using polynomial 0x8005)
        self.crc = self._crc16(data, 0x8005)
        # Append CRC as 2-byte little-endian unsigned short
        data += struct.pack("<H", self.crc)
        # Append footer byte
        data += struct.pack("<B", self.FOOTER)
        return data

    def decode(self, data):
        # Accept either list of ints or raw bytes
        if isinstance(data, list):
            data = bytes(data)
        # Minimum packet size: header(1) + counter(1) + type(1) + length(1) + crc(2) + footer(1) = 7
        if len(data) < 8:
            return False
        # Validate framing bytes
        if data[0] != self.HEADER or data[-1] != self.FOOTER:
            return False
        self.counter = data[1]
        self.msg_type = data[2]
        self.length = data[3]
        payload_end = 4 + self.length
        self.payload = data[4:payload_end]
        # Unpack CRC from the two bytes following the payload
        self.crc = struct.unpack("<H", data[payload_end:payload_end+2])[0]
        # Recompute CRC to verify data integrity
        calculated = self._crc16(list(data[:payload_end]), 0x8005)
        return self.crc == calculated

    @staticmethod
    def _crc16(data, poly=0x8005):
        # Standard CRC-16-CCITT algorithm (16-bit, polynomial 0x8005)
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
        # Factory: builds a STEERING_COMMAND packet carrying a float (servo angle in degrees)
        # and an unsigned byte (motor speed 0–255)
        pkt = Packet()
        pkt.counter = counter
        pkt.msg_type = PacketType.STEERING_COMMAND
        # servo_angle as little-endian float (4 bytes), motor_speed as unsigned byte (1 byte)
        pkt.payload = struct.pack("<fB", servo_angle, int(motor_speed))
        return pkt

    @staticmethod
    def make_emergency_stop(counter):
        # Factory: builds an EMERGENCY_STOP packet with an empty payload
        pkt = Packet()
        pkt.counter = counter
        pkt.msg_type = PacketType.EMERGENCY_STOP
        pkt.length = 0
        return pkt
