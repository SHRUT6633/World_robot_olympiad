# WRO 2026 Error Reference Catalog

## Quick Reference (sorted by severity)

| Severity | Error | Subsystem | Recovery |
|----------|-------|-----------|----------|
| CRITICAL | UART SerialException | comm/uart.py | ESP watchdog stops motors in 500ms |
| CRITICAL | Logger AttributeError | system/logger.py | Fixed in v2 — init() called at import |
| CRITICAL | Filterpy import error | fusion/ukf.py | Typo: Merked -> Merwe (fixed) |
| HIGH | I2C Errno 121 | All I2C sensors | Rate-limited + auto-disable after 50 |
| HIGH | Camera init failure | sensors/camera/ | Returns None frames; ToF-only fallback |
| HIGH | Pillar not detected | perception/ | Continues with wall-following |
| MEDIUM | UKF singular matrix | fusion/ukf.py | Skip update, use prediction only |
| MEDIUM | Parking failure | mission/ | Partial points (7/15) |
| LOW | CRC error on UART | comm/protocol.py | Packet discarded, counter incremented |
| LOW | I2C Errno 5 | I2C sensors | Same as Errno 121 handling |

## Detailed Entries

### I2C: Errno 121 Remote I/O
- **Error:** `OSError: [Errno 121] Remote I/O error`
- **When:** Any I2C read/write (MPU6050, QMC5883L, VL53L0X, VL53L1X)
- **Cause:** I2C slave did not acknowledge its address
- **Where:** pi/sensors/imu/mpu6050.py -> _read_word(), pi/sensors/tof/vl53l0x.py -> read_raw()
- **Handling:** Exception caught -> log.warn() (rate-limited, 1 per 2s) -> after 50 failures, _enabled = False
- **Severity:** HIGH if all sensors fail (bus dead), LOW if one sensor

### UART: SerialException
- **Error:** `serial.serialutil.SerialException: device reports readiness to read but returned no data`
- **When:** UART read/write in comm_task
- **Cause:** ESP32 not powered, wrong baud rate, or wrong /dev/serial0
- **Where:** pi/comm/uart.py -> read() / write()
- **Handling:** Returns None -> no motor commands -> ESP watchdog stops motors after 500ms
- **Severity:** CRITICAL

### UKF: Singular Matrix
- **Error:** `np.linalg.LinAlgError: Singular matrix`
- **When:** UKF update() — covariance matrix inversion
- **Cause:** All measurements are zero/identical; filter not initialised with valid P
- **Where:** pi/fusion/ukf.py -> update()
- **Handling:** Exception caught -> skip update -> state uncertainty grows -> recovers when valid measurements arrive
- **Severity:** MEDIUM
