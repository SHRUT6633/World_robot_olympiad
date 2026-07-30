import pytest
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pi.comm.protocol import Packet, PacketType


class TestComm:
    def test_packet_encode_decode(self):
        pkt = Packet.make_steering_command(1, 15.0, 75)
        data = pkt.encode()
        decoded = Packet()
        assert decoded.decode(list(data))
        assert decoded.msg_type == PacketType.STEERING_COMMAND
        assert decoded.counter == 1

    def test_emergency_stop(self):
        pkt = Packet.make_emergency_stop(0)
        data = pkt.encode()
        decoded = Packet()
        assert decoded.decode(list(data))
        assert decoded.msg_type == PacketType.EMERGENCY_STOP

    def test_crc_detects_corruption(self):
        pkt = Packet.make_steering_command(0, 10.0, 50)
        data = bytearray(pkt.encode())
        data[5] ^= 0xFF  # corrupt payload
        decoded = Packet()
        assert not decoded.decode(list(data))

    def test_packet_types(self):
        assert int(PacketType.MOTOR_COMMAND) == 0x01
        assert int(PacketType.EMERGENCY_STOP) == 0xFF
