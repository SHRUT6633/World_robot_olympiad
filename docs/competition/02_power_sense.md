# 2. Power and Sense Management (4 pts)

## Power System
- **Battery 1**: Powers Raspberry Pi (5V via regulator) + ESP32 (5V)
- **Battery 2**: Powers DC motor + servo (7.4V LiPo)
- Voltage stabiliser prevents SBC brownout during motor spikes

## Sensors
| Sensor | Qty | Protocol | Address | Purpose |
|--------|-----|----------|---------|---------|
| PiCamera | 1 | CSI-2 | /dev/video0 | Lane + pillar detection |
| VL53L0X | 2 | I2C | 0x30, 0x31 | Left/right wall distance |
| VL53L1X | 1 | I2C | 0x32 | Forward obstacle distance |
| MPU6050 | 1 | I2C | 0x68 | 6-DoF IMU (accel + gyro) |
| QMC5883L | 1 | I2C | 0x0D | Magnetometer (heading) |

## Sensor Fusion Pipeline
```
Camera → PillarDetector (color) + LaneDetector (edges)
ToF    → WallDetector (distance flags)
IMU    → ComplementaryFilter (pitch/roll) → UKF (6-DoF state)
Mag    → Heading correction
                  ↓
         RobotLocalization (pose)
```

## Files
- `pi/sensors/camera/` — Camera drivers
- `pi/sensors/tof/` — VL53L0X + VL53L1X drivers
- `pi/sensors/imu/` — MPU6050 driver
- `pi/sensors/magnetometer/` — QMC5883L driver
- `pi/fusion/` — UKF + complementary filter + adaptive noise
- `pi/localization/` — Pose estimation + localization
