# v1.6 — Multi-Sensor Read Test

## Reading All Sensors in a Loop

With individual sensor tests passing (I2C in v1.1, camera in v1.2), our next goal was to read all five I2C sensors simultaneously in a single loop at 100Hz. The sensors are: MPU6050 accelerometer/gyroscope (6-axis IMU), QMC5883L magnetometer (3-axis compass), two VL53L0X time-of-flight sensors (left and right short-range, 30-2000mm), and one VL53L1X time-of-flight sensor (front long-range, 40-4000mm). Each sensor communicates over the same I2C bus but has a unique address.

The 100Hz target rate was chosen to match our control loop frequency. The robot's PID controller needs fresh sensor data every 10ms to make timely steering corrections. If sensor reads take longer than 10ms, the controller would be working with stale data, reducing the robot's ability to follow the line accurately.

## First Error: I2C Bus Contention

When we ran the multi-sensor loop at full speed, we encountered I2C bus contention errors. The smbus2 library raised IOError exceptions with messages like "Remote I/O error" and "Transaction failed." These errors occurred intermittently but became more frequent when we added more sensors to the loop.

The root cause was that different sensors have different response times. The MPU6050 can respond within 100μs after receiving the register address, because its accelerometer and gyroscope data registers are updated continuously at 1kHz and are always ready to read. The QMC5883L similarly has a data-ready rate of up to 200Hz, so reads within 5ms of each other succeed. However, the VL53L0X time-of-flight sensors have a fundamentally different operating principle: they measure distance by emitting a laser pulse and timing its reflection. This measurement takes approximately 30ms per reading (including the laser pulse, photon integration, and signal processing). If we try to read the VL53L0X's measurement register before the measurement is complete, the sensor returns either stale data or an error code indicating "measurement not ready."

When we looped through all five sensors back-to-back without any delay, the VL53L0X sensors would return errors because we were reading faster than their 30ms measurement cycle. The errors then caused I2C bus lockups, affecting reads from the other sensors as well.

## The Fix: Staggered Reads with 10ms Delay

We implemented two fixes. First, we added a 10ms delay between each sensor read using `time.sleep(0.01)`. This gives the bus time to settle and prevents back-to-back transactions from overloading the I2C controller. Second, we staggered the sensor reads so that the fast sensors (MPU6050, QMC5883L) are read on every loop iteration, while the slow sensors (VL53L0X, VL53L1X) are read on alternating iterations.

The staggered approach works like this:

```
Loop iteration 1: MPU6050, QMC5883L, VL53L0X_L, VL53L0X_R, VL53L1X  (all sensors)
Loop iteration 2: MPU6050, QMC5883L                                    (fast only)
Loop iteration 3: MPU6050, QMC5883L, VL53L0X_L, VL53L0X_R, VL53L1X  (all sensors)
```

This gives the ToF sensors 20ms between reads, which is still within their 30ms measurement cycle. The MPU6050 and QMC5883L are read every 10ms, providing 100Hz update rate for the IMU data that the control loop needs most urgently.

## The SensorBase Class

To manage the different timing requirements, we created a `SensorBase` Python class that wraps each sensor with timeout and error counting:

```python
class SensorBase:
    def __init__(self, addr, name, timeout_ms=100):
        self.addr = addr
        self.name = name
        self.timeout = timeout_ms / 1000.0
        self.error_count = 0
        self.last_read_time = 0

    def read(self):
        if time.time() - self.last_read_time < self.timeout:
            return None  # Not yet due for reading
        try:
            data = bus.read_i2c_block_data(self.addr, ...)
            self.last_read_time = time.time()
            return data
        except Exception as e:
            self.error_count += 1
            return None
```

Each sensor has a configurable timeout (minimum time between reads). The `read()` method returns `None` if the sensor is not yet due for a reading, allowing the control loop to call `read()` for all sensors every iteration without worrying about timing. Sensors that exceed their error count threshold (e.g., 10 consecutive errors) are flagged as failed in the self-test.

## Alternative: Separate I2C Buses

The Raspberry Pi 4 has two I2C buses: bus 0 (usually reserved for HAT identification via ID EEPROM) and bus 1 (available on GPIO 2 and 3). We considered splitting the sensors across both buses to reduce contention. For example, the MPU6050 and QMC5883L could be on bus 0, while the ToF sensors could be on bus 1. This would allow parallel reads and eliminate the need for staggering.

However, using both buses would require additional wiring: bus 0 uses GPIO 0 (SDA) and GPIO 1 (SCL), which are different from bus 1's GPIO 2 and 3. We would need to run two separate I2C cables from the Pi to the sensor board, adding weight and complexity. More importantly, the ESP32's I2C controller would need to be connected to both buses, which would require two sets of I2C pins on the ESP32 as well. The hardware complexity was not justified by the modest performance gain.

## Learned: VL53L0X Measurement Time

The biggest discovery in v1.6 was the VL53L0X's 30ms measurement time. The datasheet claims a "typical measurement time" of 30ms in standard mode, but this can be reduced to 20ms in "fast mode" (at the cost of reduced accuracy) or increased to 60ms in "long range mode" (for better accuracy at distances beyond 1m). We chose the standard mode as a compromise between update rate and accuracy.

At 30ms per measurement, each ToF sensor can deliver at most 33 readings per second. With three ToF sensors (left, right, front), we can read each one at approximately 11Hz if we cycle through them. This is sufficient for obstacle detection — a robot moving at 0.5 m/s travels only 4.5cm between ToF readings, which is acceptable for collision avoidance.

## I2C Clock Stretching

One subtle issue we encountered was I2C clock stretching. The VL53L0X sensor can hold the SCL line low (clock stretching) while it processes a measurement. If the I2C master (the Pi or ESP32) does not support clock stretching, the transaction will fail. Fortunately, the Raspberry Pi's I2C controller supports clock stretching up to the specification limit of 150ms. We verified that the VL53L0X's clock stretching never exceeds 50ms during normal operation. However, if the VL53L0X is in the middle of a long-distance measurement (60ms long-range mode) and we try to read it, the clock stretching can approach 60ms, which is still within the Pi's limit. We chose to use the standard 30ms measurement mode to keep clock stretching predictable.

## Sensor Data Format and Scaling

Each sensor returns data in a different format. The MPU6050 returns 16-bit signed integers for each axis of acceleration and gyroscope. The accelerometer full-scale range is ±2g by default (we set it to ±2g for maximum sensitivity), so each bit represents 1/16384 g. The gyroscope full-scale range is ±250°/s by default, so each bit represents 1/131.072 °/s. These scaling factors must be applied in software to convert raw ADC values to physical units. The QMC5883L returns 16-bit signed integers for magnetometer readings, with a default range of ±8 gauss and a sensitivity of 3000 LSB/gauss. The VL53L0X and VL53L1X return distance in millimeters as a 16-bit unsigned integer. We standardize all sensor outputs to floating-point values in SI units (m/s², °/s, gauss, meters) before passing them to the sensor fusion algorithm.

## Error Counting and Fault Tolerance

Each sensor read is wrapped in a try/except block that catches `IOError`, `OSError`, and any other exceptions. Failed reads increment an error counter, while successful reads reset it to zero. If any sensor accumulates 10 consecutive errors, the self-test system flags it as "degraded" but continues running. Certain failures (e.g., MPU6050 gyro failure) are critical because the control loop depends on gyro data for stability. The risk assessment in v1.9 documents which sensors are critical and which are optional.

In practice, we observed approximately 0.1% error rate on the I2C bus under normal conditions, increasing to 1% when the motors are running (electrical noise coupling into the I2C wires). The try/except approach handles these transient errors gracefully: the control loop uses the previous valid reading until a new one is available, rather than crashing on the first error.
