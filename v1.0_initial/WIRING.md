# WRO 4WS Wiring / Pinout

## Raspberry Pi 4B GPIO

| Pin | Function       | Connected To        |
|-----|----------------|---------------------|
| 23  | Green LED      | LED+ (100R)         |
| 24  | Red LED        | LED+ (100R)         |
| 25  | Start Switch   | Switch (GND pull-up)|
| 8   | UART TX        | ESP32 RX (GPIO18)   |
| 10  | UART RX        | ESP32 TX (GPIO17)   |
| 3   | I2C SDA        | All I2C sensors     |
| 5   | I2C SCL        | All I2C sensors     |
| 1   | 3.3V           | Sensors power       |
| 6   | GND            | Common ground       |

## ESP32-S3 GPIO (Custom Board)

| Pin | Function       | Connected To        |
|-----|----------------|---------------------|
| 2   | Green LED      | Onboard status OK   |
| 4   | Red LED        | Onboard error       |
| 13  | Servo PWM      | MG995 signal wire   |
| 11  | Motor PWM A    | TB6612FNG PWMA      |
| 12  | Motor PWM B    | TB6612FNG PWMB      |
| 10  | STBY           | TB6612FNG STBY      |
| 8   | AIN1           | TB6612FNG AIN1      |
| 9   | AIN2           | TB6612FNG AIN2      |
| 6   | BIN1           | TB6612FNG BIN1      |
| 7   | BIN2           | TB6612FNG BIN2      |
| 17  | UART TX        | Pi RX (GPIO10)      |
| 18  | UART RX        | Pi TX (GPIO8)       |

## I2C Sensor Addresses

| Sensor              | Address |
|---------------------|---------|
| VL53L0X (Left)      | 0x30    |
| VL53L0X (Right)     | 0x31    |
| VL53L1X (Front)     | 0x32    |
| MPU6050             | 0x68    |
| QMC5883L            | 0x0D    |

## LED Boot Sequence

| Phase              | Green LED | Red LED |
|--------------------|-----------|---------|
| Power ON           | Blink     | Blink   |
| Running self-test  | OFF       | OFF     |
| ALL TESTS PASSED   | ON steady | OFF     |
| TEST FAILED        | OFF       | Blink   |
| Switch pressed     | OFF       | OFF     |
| Race logic active  | ON dim    | OFF     |
| Error / Comm loss  | OFF       | ON      |
| Emergency stop     | OFF       | ON blink|

## Start Switch

Momentary push button between GPIO25 and GND (internal pull-up enabled).
Press after green LED is steady to begin race logic.
