import time

DISABLE_THRESHOLD = 50
RATE_LIMIT_SEC = 2.0
STARTUP_GRACE_SEC = 3.0

class SensorHealthMonitor:
    def __init__(self, sensors=None):
        if sensors is None:
            sensors = ["imu", "mag", "tof_left", "tof_right", "tof_front", "camera"]
        self.sensors = sensors
        self._consecutive_failures = {s: 0 for s in sensors}
        self._prev_success = {s: True for s in sensors}
        self._disabled = {s: False for s in sensors}
        self._last_log = {s: 0.0 for s in sensors}
        self._start_monotonic = time.monotonic()
        self._startup_deadline = self._start_monotonic + STARTUP_GRACE_SEC

    def report_success(self, sensor):
        if sensor not in self.sensors:
            return
        self._consecutive_failures[sensor] = 0
        self._prev_success[sensor] = True

    def report_failure(self, sensor, error_msg=""):
        if sensor not in self.sensors:
            return
        if self._disabled[sensor]:
            return
        if time.monotonic() < self._startup_deadline:
            return

        if self._prev_success[sensor]:
            self._consecutive_failures[sensor] = 1
        else:
            self._consecutive_failures[sensor] += 1

        self._prev_success[sensor] = False

        now = time.monotonic()
        if now - self._last_log[sensor] >= RATE_LIMIT_SEC:
            print(f"ERROR: {sensor}: {error_msg} "
                  f"({self._consecutive_failures[sensor]}/{DISABLE_THRESHOLD})")
            self._last_log[sensor] = now

        if self._consecutive_failures[sensor] >= DISABLE_THRESHOLD:
            self._disabled[sensor] = True
            print(f"WARNING: {sensor} disabled after "
                  f"{DISABLE_THRESHOLD} consecutive failures")

    def status(self):
        result = {}
        for s in self.sensors:
            if self._disabled[s]:
                result[s] = "disabled"
            elif self._consecutive_failures[s] > 0:
                result[s] = "warning"
            else:
                result[s] = "ok"
        return result

    def should_read(self, sensor):
        return not self._disabled.get(sensor, True)

    def reset(self, sensor):
        if sensor in self.sensors:
            self._consecutive_failures[sensor] = 0
            self._prev_success[sensor] = True
            self._disabled[sensor] = False

if __name__ == "__main__":
    monitor = SensorHealthMonitor()
    print("Simulating sensor failures...")

    for i in range(60):
        if i < 55:
            monitor.report_failure("tof_left", "OSError(121, 'Remote I/O error')")
        else:
            monitor.report_success("tof_left")
        monitor.report_success("imu")
        time.sleep(0.03)

    print(f"Status: {monitor.status()}")
    print(f"  tof_left should_read: {monitor.should_read('tof_left')}")
    print(f"  imu should_read: {monitor.should_read('imu')}")
