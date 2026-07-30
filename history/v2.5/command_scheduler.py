import threading
import queue
import json
import time

class CommandScheduler:
    def __init__(self, uart):
        self.uart = uart
        self.cmd_queue = queue.Queue()
        self.running = False
        self.thread = threading.Thread(target=self._sender_loop, daemon=True)

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False

    def send(self, cmd, **kwargs):
        self.cmd_queue.put((cmd, kwargs))

    def _sender_loop(self):
        while self.running:
            try:
                cmd, kwargs = self.cmd_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            msg = json.dumps({"cmd": cmd, **kwargs}) + '\n'
            self.uart.write(msg.encode())
