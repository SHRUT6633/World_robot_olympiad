import serial
import json

MSG_TERM = '\n'

def encode_command(cmd, **kwargs):
    msg = {"cmd": cmd}
    msg.update(kwargs)
    return json.dumps(msg) + MSG_TERM

def send_command(uart, cmd, **kwargs):
    uart.write(encode_command(cmd, **kwargs).encode())

class UARTProtocol:
    def __init__(self, port, baud=115200):
        self.uart = serial.Serial(port, baud, timeout=0.05)

    def send(self, cmd, **kwargs):
        send_command(self.uart, cmd, **kwargs)

    def read_response(self):
        line = self.uart.readline()
        if line:
            try:
                return json.loads(line.decode().strip())
            except json.JSONDecodeError:
                return None
        return None

    def close(self):
        self.uart.close()
