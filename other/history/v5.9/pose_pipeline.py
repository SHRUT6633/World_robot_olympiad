import threading
import time
import numpy as np
from outlier_reject import OutlierRejectUKF
from cross_verify import CrossVerify
from mag_heading import MagHeading


class PosePipeline:
    def __init__(self):
        self.ukf = OutlierRejectUKF()
        self.verify = CrossVerify()
        self.mag = MagHeading()
        self._lock = threading.Lock()
        self._running = False
        self._pending_z = []
        self.loop_hz = {"predict": 0, "correct": 0, "verify": 0}

    def start(self):
        self._running = True
        threading.Thread(target=self._fast_loop, daemon=True).start()
        threading.Thread(target=self._medium_loop, daemon=True).start()
        threading.Thread(target=self._slow_loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _fast_loop(self):
        while self._running:
            t0 = time.monotonic()
            encoders = self._read_encoders()
            gyro = self._read_gyro()
            dt = 0.01
            with self._lock:
                self.ukf.predict(dt)
            elapsed = time.monotonic() - t0
            self.loop_hz["predict"] = 1.0 / max(elapsed, 1e-6)
            if elapsed > 0.009:
                print(f"[WARN] Predict took {elapsed*1000:.1f}ms")
            time.sleep(max(0.01 - elapsed, 0.001))

    def _medium_loop(self):
        while self._running:
            t0 = time.monotonic()
            z = self._gather_measurements()
            if z is not None:
                with self._lock:
                    self.ukf.correct(z)
            elapsed = time.monotonic() - t0
            self.loop_hz["correct"] = 1.0 / max(elapsed, 1e-6)
            time.sleep(max(0.02 - elapsed, 0.001))

    def _slow_loop(self):
        while self._running:
            t0 = time.monotonic()
            tof = self._read_tof()
            cam = self._read_camera()
            motor_on = self._motor_running()
            self.verify.verify(tof, cam, self.ukf.ukf.x[2], 0.0)
            self.mag.update(0.0, 0.0, motor_on)
            elapsed = time.monotonic() - t0
            self.loop_hz["verify"] = 1.0 / max(elapsed, 1e-6)
            time.sleep(max(0.05 - elapsed, 0.001))

    def _read_encoders(self):
        return np.random.randn(2) * 0.001

    def _read_gyro(self):
        return np.random.randn(1) * 0.01

    def _read_tof(self):
        return 1.0 + np.random.randn() * 0.02

    def _read_camera(self):
        return np.array([0.0, 0.0, 0.0])

    def _gather_measurements(self):
        return np.array([0.0, 0.0, 0.0])

    def _motor_running(self) -> bool:
        return False

    def pose(self) -> tuple:
        with self._lock:
            return tuple(self.ukf.ukf.x[:3])

    def report(self) -> str:
        return (f"predict={self.loop_hz['predict']:.0f}Hz "
                f"correct={self.loop_hz['correct']:.0f}Hz "
                f"verify={self.loop_hz['verify']:.0f}Hz")
