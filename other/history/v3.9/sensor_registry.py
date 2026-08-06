SENSORS = [
    "imu",
    "mag",
    "tof_left",
    "tof_right",
    "tof_front",
    "camera",
]

DISABLE_THRESHOLD = 50
RATE_LIMIT_SEC = 2.0
STARTUP_GRACE_SEC = 3.0

DEFAULT_VALUES = {
    "imu": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0},
    "mag": 0.0,
    "tof_left": 2000,
    "tof_right": 2000,
    "tof_front": 4000,
    "camera": None,
}

SENSOR_PRIORITY = {
    "imu": 1,
    "tof_front": 2,
    "tof_left": 3,
    "tof_right": 3,
    "mag": 4,
    "camera": 5,
}
