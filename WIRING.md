# WRO 4WS Wiring / Pinout (BCM numbering)

## Raspberry Pi 4B GPIO

| BCM | Phys | Function       | Connected To            | Config Source                    |
|-----|------|----------------|-------------------------|----------------------------------|
| 14  | 8    | UART TX        | ESP32 RX (GPIO18)       | `config/pi_config.yaml` → `/dev/serial0` |
| 15  | 10   | UART RX        | ESP32 TX (GPIO17)       | `config/pi_config.yaml` → `/dev/serial0` |
| 2   | 3    | I2C1 SDA       | All I2C sensors         | Standard Pi 4B I2C-1 bus         |
| 3   | 5    | I2C1 SCL       | All I2C sensors         | Standard Pi 4B I2C-1 bus         |
| 17  | 11   | VL53L0X Left XSHUT  | ToF Left XSHUT pin | `config/pi_config.yaml` → `sensors.vl53l0x_left.xshut_pin` |
| 27  | 13   | VL53L0X Right XSHUT | ToF Right XSHUT pin| `config/pi_config.yaml` → `sensors.vl53l0x_right.xshut_pin` |
| 22  | 15   | VL53L1X Front XSHUT | ToF Front XSHUT pin| `config/pi_config.yaml` → `sensors.vl53l1x_front.xshut_pin` |
| 23  | 16   | Green LED      | LED+ (100R)             | `config/pi_config.yaml` → `hardware.leds.green_pin` |
| 24  | 18   | Red LED        | LED+ (100R)             | `config/pi_config.yaml` → `hardware.leds.red_pin` |
| 25  | 22   | Start Switch   | Switch (GND, pull-up)   | `config/pi_config.yaml` → `hardware.switch.pin` |
| 1   | 1    | 3.3V           | Sensors power           | —                                |
| 6   | 6    | GND            | Common ground           | —                                |
| 14  | 25   | GND            | Common ground           | —                                |
| 39  | 39   | GND            | Common ground           | —                                |

## ESP32-S3 GPIO (Custom Board)

| GPIO | Function       | Connected To               | Code Reference                  |
|------|----------------|----------------------------|---------------------------------|
| 17   | UART TX        | Pi RX (BCM 15)             | `esp/main/main.c` → `UART_TX_GPIO` |
| 18   | UART RX        | Pi TX (BCM 14)             | `esp/main/main.c` → `UART_RX_GPIO` |
| 2    | Green LED      | Onboard status OK          | `esp/main/main.c` → `LED_GREEN_GPIO` |
| 4    | Red LED        | Onboard error              | `esp/main/main.c` → `LED_RED_GPIO` |
| 13   | Servo PWM      | MG995 signal wire          | `esp/main/servo_pwm.c` → `SERVO_PIN` |
| 11   | ENA            | L298N ENA (PWM)            | `esp/main/l298n.c` → `PIN_ENA` |
| 8    | IN1            | L298N IN1 (direction)      | `esp/main/l298n.c` → `PIN_IN1` |
| 9    | IN2            | L298N IN2 (direction)      | `esp/main/l298n.c` → `PIN_IN2` |

## Wiring Diagram

```
Pi (BCM)                          ESP32-S3 (GPIO)
────────                          ──────────────
BCM 14 (UART TX) ──────────────── GPIO 18 (UART RX)
BCM 15 (UART RX) ──────────────── GPIO 17 (UART TX)
GND (phys 6)     ──────────────── GND
```

## I2C Sensor Addresses

| Sensor              | Address | XSHUT BCM Pin |
|---------------------|---------|---------------|
| VL53L0X (Left)      | 0x30    | 17            |
| VL53L0X (Right)     | 0x31    | 27            |
| VL53L1X (Front)     | 0x32    | 22            |
| MPU6050             | 0x68    | —             |
| QMC5883L            | 0x0D    | —             |

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

Momentary push button between BCM 25 (phys 22) and GND (internal pull-up enabled).
Press after green LED is steady to begin race logic.