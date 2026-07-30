# System Architecture

## Data Flow Diagram
```
+------------------------------------------------------------------+
| RASPBERRY PI 4                                                    |
|                                                                    |
|  CAMERA -> LANE_DET -> PILLAR_DET -> PARKING_DET                  |
|    |          |             |              |                       |
|    +----------+------+------+--------+     |                       |
|                       |                 |                          |
|  TOF_L + TOF_R -> WALL_DET             |                          |
|  TOF_F -> OBSTACLE_DET                 |                          |
|  IMU + MAG -> COMPLEMENTARY_FILTER     |                          |
|                      |                 |                          |
|                      v                 v                          |
|                 UKF (6-DoF) -> LOCALIZATION -> STATE_MACHINE       |
|                                                  |                 |
|  GLOBAL_PLANNER <- WAYPOINT_GEN <- TRAJECTORY <-+                 |
|       |                                                           |
|       v                                                           |
|  STANLEY + PID -> UART COMM -> ESP32                              |
+------------------------------------------------------------------+
                                 |
                              UART @115200
                                 |
+------------------------------------------------------------------+
| ESP32-S3                                                          |
|  UART_RX -> CRC_CHECK -> COMMAND_DISPATCH                         |
|                               |                                   |
|                    +----------+--------+                          |
|                    |                   |                          |
|               SERVO_PWM            L298N_MOTOR                    |
|                    |                   |                          |
|               STEERING             DRIVE                          |
|                                                                    |
|  WATCHDOG(3s) + TIMEOUT(500ms) + FAILSAFE                         |
+------------------------------------------------------------------+
```

## Task Rates
| Task | Hz | Priority |
|------|----|----------|
| sensors | 100 | 10 |
| fusion | 100 | 9 |
| perception | 50 | 8 |
| planning | 20 | 7 |
| control | 100 | 10 |
| comms | 200 | 9 |
| health | 2 | 0 |

## Key Design Decisions
1. No SLAM — geometry-based track map (faster, lighter)
2. Single-loop callbacks — scheduler owns timing via hz
3. Rate-limited error logging — 1 per 2s, auto-disable after 50
4. Three steering modes — SAME_PHASE, OPPOSITE_PHASE, CRAB_WALK
5. Surprise rule config — change one line, no code edits
6. Exact rulebook colours — RGB -> HSV via calibration
7. Parking verification — 30s stationary + parallel <= 2cm
