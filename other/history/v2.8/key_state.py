import threading
from pynput import keyboard

class KeyState:
    def __init__(self):
        self.keys = set()
        self.lock = threading.Lock()
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )

    def start(self):
        self.listener.start()

    def stop(self):
        self.listener.stop()

    def _on_press(self, key):
        with self.lock:
            self.keys.add(self._key_id(key))

    def _on_release(self, key):
        with self.lock:
            self.keys.discard(self._key_id(key))

    def _key_id(self, key):
        if hasattr(key, 'vk') and key.vk is not None:
            return key.vk
        if hasattr(key, 'char') and key.char is not None:
            return ord(key.char)
        return hash(key)

    def is_pressed(self, key_code):
        with self.lock:
            return key_code in self.keys

    def any_pressed(self, *key_codes):
        with self.lock:
            return any(k in self.keys for k in key_codes)
