# 2. Power and Sense Management (4 pts)

## Power System
- **Battery 1 (5V):** Powers Raspberry Pi 4 via regulator + ESP32-S3 via 5V rail
- **Battery 2 (7.4V LiPo):** Powers DC motor + steering servo (isolated from SBC)
- Voltage stabiliser prevents brownout during motor current spikes
- **Config:** `config/pi_config.yaml:1-10` — Power rail configuration

## Sensors
| Sensor | Qty | Protocol | Address | Purpose | Code Location |
|--------|-----|----------|---------|---------|--------------|
| PiCamera | 1 | CSI-2 | /dev/video0 | Lane + pillar detection | `pi/sensors/camera/` |
| VL53L0X | 2 | I2C | 0x30, 0x31 | Left/right wall distance | `pi/sensors/tof/vl53l0x.py:10` |
| VL53L1X | 1 | I2C | 0x32 | Forward obstacle distance | `pi/sensors/tof/vl53l1x.py:12` |
| MPU6050 | 1 | I2C | 0x68 | 6-DoF IMU (accel + gyro) | `pi/sensors/imu/mpu6050.py:15` |
| QMC5883L | 1 | I2C | 0x0D | Magnetometer (heading) | `pi/sensors/magnetometer/qmc5883l.py:8` |

## Sensor Fusion Pipeline
- **File/line:** `pi/fusion/ukf.py:30` — RobotUKF class (6-DoF Unscented Kalman Filter)
- **File/line:** `pi/fusion/complementary.py:5` — ComplementaryFilter for drift-free pitch/roll
- **File/line:** `pi/fusion/adaptive_noise.py:10` — AdaptiveNoiseEstimator tunes process noise
- **File/line:** `pi/localization/robot_localization.py:20` — RobotLocalization holds final pose

## Fusion Data Flow
```
Camera -> PillarDetector (color) + LaneDetector (edges)
ToF    -> WallDetector (distance flags)
IMU    -> ComplementaryFilter (pitch/roll) -> UKF (6-DoF state)
Mag    -> Heading correction
                  -->
         RobotLocalization (pose) -> StanleyController
```

## I2C Error Handling
- **File/line:** `pi/sensors/base.py:25` — Rate-limited error logging (max 1 per 2s)
- **File/line:** `pi/sensors/base.py:45` — Auto-disable after 50 consecutive failures
