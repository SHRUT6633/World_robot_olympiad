from collections import deque
from ..system.logger import log


class SensorBufferManager:
    def __init__(self, maxlen=100):
        self._buffers = {}
        self.maxlen = maxlen

    def create(self, name, maxlen=None):
        self._buffers[name] = deque(maxlen=maxlen or self.maxlen)

    def push(self, name, data):
        if name not in self._buffers:
            self.create(name)
        self._buffers[name].append(data)

    def latest(self, name):
        buf = self._buffers.get(name)
        return buf[-1] if buf else None

    def all_latest(self):
        return {n: buf[-1] if buf else None for n, buf in self._buffers.items()}

    def clear(self, name=None):
        if name:
            self._buffers.get(name, deque()).clear()
        else:
            for b in self._buffers.values():
                b.clear()

    def __getitem__(self, name):
        return list(self._buffers.get(name, []))
