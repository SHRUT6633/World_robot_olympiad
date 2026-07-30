from .runner import SelfTestRunner, TestResult


def register_comm_tests(runner: SelfTestRunner, uart):
    # Register six communication self-tests:
    #   1. packet_encode   -- encode a steering command Packet and check size.
    #   2. packet_decode   -- round-trip encode + decode, verify fields match.
    #   3. crc_detection   -- corrupt a byte and verify CRC catches it.
    #   4. uart_connect    -- open the UART serial port.
    #   5. uart_esp_echo   -- send a packet over UART (no response check).
    #   6. esp_heartbeat   -- check if the ESP32 is responding.
    #
    # uart -- a serial/UART abstraction, typically from pi.comm.uart_comm.

    def test_packet_encode():
        # Test: build a steering command Packet and encode to bytes.
        # The encoded data must be at least 8 bytes (header + payload + CRC).
        from ..comm.protocol import Packet

        pkt = Packet.make_steering_command(0, 15.0, 75)
        data = pkt.encode()
        if len(data) < 8:
            return TestResult("packet_encode").failed(
                f"Packet too short: {len(data)} bytes"
            )
        return TestResult("packet_encode").passed(
            f"{len(data)} bytes, CRC=0x{pkt.crc:04X}"
        )

    def test_packet_decode():
        # Test: encode a known packet, decode it, and verify message type
        # and CRC are preserved.
        from ..comm.protocol import Packet

        original = Packet.make_steering_command(1, -10.5, 50)
        data = original.encode()
        decoded = Packet()
        if not decoded.decode(list(data)):
            return TestResult("packet_decode").failed("Decode failed")
        if decoded.msg_type != original.msg_type:
            return TestResult("packet_decode").failed("Type mismatch")
        if decoded.crc != original.crc:
            return TestResult("packet_decode").failed("CRC mismatch")
        return TestResult("packet_decode").passed(
            f"type=0x{decoded.msg_type:02X} counter={decoded.counter}"
        )

    def test_crc_detection():
        # Test: corrupt one byte of an otherwise valid packet and verify
        # that decode() returns False (CRC failure).
        from ..comm.protocol import Packet

        pkt = Packet.make_steering_command(0, 10.0, 50)
        data = bytearray(pkt.encode())
        data[5] ^= 0xFF  # Flip all bits in byte 5.
        decoded = Packet()
        if decoded.decode(list(data)):
            return TestResult("crc_detection").failed(
                "Should have detected corruption"
            )
        return TestResult("crc_detection").passed("CRC detected corrupted packet")

    def test_uart_connect():
        # Test: initialise the UART connection.
        if uart is None or uart._serial is None:
            return TestResult("uart_connect").skipped("UART not available")
        uart.init()
        if not hasattr(uart, "_serial") or uart._serial is None:
            return TestResult("uart_connect").failed("UART init failed")
        return TestResult("uart_connect").passed(f"Port {uart.port} @ {uart.baudrate}")

    def test_uart_esp_echo():
        # Test: construct and send a zero-steering packet over UART.
        # This only tests that the send call succeeds; it does not verify
        # the ESP32 actually received it (use heartbeat for that).
        if uart is None or uart._serial is None:
            return TestResult("uart_esp_echo").skipped("UART not available")
        from ..comm.protocol import Packet

        pkt = Packet.make_steering_command(0, 0.0, 0)
        if not uart.send(pkt):
            return TestResult("uart_esp_echo").failed("Send failed")
        return TestResult("uart_esp_echo").passed(
            "Packet sent, awaiting ESP32 response"
        )

    def test_esp_heartbeat():
        # Test: check the ESP32 communication link via the is_connected flag.
        if uart is None:
            return TestResult("esp_heartbeat").skipped("UART not available")
        connected = uart.is_connected
        if connected:
            return TestResult("esp_heartbeat").passed("ESP32 communicating")
        return TestResult("esp_heartbeat").failed(
            "No ESP32 heartbeat - check connection"
        )

    runner.add("packet_encode", test_packet_encode)
    runner.add("packet_decode", test_packet_decode)
    runner.add("crc_detection", test_crc_detection)
    runner.add("uart_connect", test_uart_connect)
    runner.add("uart_esp_echo", test_uart_esp_echo)
    runner.add("esp_heartbeat", test_esp_heartbeat)
