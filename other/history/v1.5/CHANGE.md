# v1.5 — UART Loopback Test

## Establishing Pi-to-ESP32 Communication

The UART (Universal Asynchronous Receiver-Transmitter) link is the backbone of communication between the Raspberry Pi and the ESP32-S3. Every command from the Pi — motor speed, servo angle, sensor request — travels over this serial connection. Every response from the ESP32 — sensor readings, status updates, acknowledgments — comes back over the same wires. The reliability of this link is therefore critical to the robot's operation.

Our test was simple: the Pi sends "ping\n" over UART, and the ESP32 replies with "pong\n". This ping-pong protocol verifies that both devices can transmit and receive, that the baud rate is correctly configured on both ends, and that the data is not corrupted during transmission. We used a text-based protocol (ASCII, newline-delimited) rather than binary encoding because text is human-readable for debugging: if something goes wrong, we can connect a serial terminal and see exactly what bytes are being exchanged.

## First Error: Lost First Byte

The first test result was puzzling. On the Pi side, we sent "ping\n" but received nothing back. On the ESP32 side, we saw that a byte was received (the UART interrupt fired), but the byte was not the ASCII character 'p' (0x70). Instead, it was either 0xFF (255) or 0x00 (null). These bytes are clearly not part of our protocol, so the ESP32's string comparison `strcmp(buf, "ping\n")` failed, and no response was sent.

The source of the garbage bytes was the ESP32's bootloader. When the ESP32 resets (either from a power-on or after flashing new firmware), the ROM bootloader prints diagnostic information over UART0 at 115200 baud. The bootloader output includes the chip's MAC address, flash size, and other debug information. This output is printed before the user's application starts. Additionally, the bootloader may leave the UART FIFO in an unknown state, with residual bytes from the boot process.

When the Pi sends "ping\n", the ESP32's UART hardware receives the byte, but the FIFO already contains the stale bootloader bytes at the front. The `uart_read_bytes()` function reads the FIFO in order, so it gets the stale byte first, then the 'p' of "ping" second. The first read returns only the stale byte, and the second read (or a subsequent read) returns "ping\n" but by then the string comparison against an incomplete buffer fails.

## The Fix: Flush UART Buffer and Add Startup Delay

We fixed the issue with two changes on the ESP32 side. First, we added `uart_flush(UART_NUM)` immediately after installing the UART driver. This clears the hardware FIFO and any residual bytes from the bootloader. Second, we added a 100ms delay (`vTaskDelay(pdMS_TO_TICKS(100))`) after the flush to ensure that the UART hardware and the connected Pi have both stabilized before we start reading.

```c
uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);
uart_flush(UART_NUM);  // Crucial: clear bootloader garbage
vTaskDelay(pdMS_TO_TICKS(100));
```

On the Pi side, we also added a flush of the input buffer after opening the serial port:

```python
ser.reset_input_buffer()  # Flush bootloader garbage
```

This ensures that any bytes the ESP32's bootloader printed during its startup are discarded before the Pi sends its first command. The combination of both flushes ensures a clean communication channel.

## Alternative: Hardware Flow Control

We considered using hardware flow control (RTS/CTS) to prevent the UART FIFO from overflowing. With hardware flow control, the receiver asserts CTS (Clear To Send) when its FIFO has space, and the transmitter waits until CTS is asserted before sending. This would prevent the receiver's FIFO from filling up and losing bytes.

However, hardware flow control requires two additional wires (RTS and CTS), which we do not have in our 4-wire UART cable (power, ground, TX, RX). Adding two more wires would require a larger cable and more GPIO pins on both boards. The ESP32-S3 has limited GPIO availability (many pins are used for the camera, I2C sensors, motor driver, and servo). We decided that hardware flow control was unnecessary for our baud rate and message size: at 115200 baud, a 10-byte message takes less than 1ms to transmit, and the ESP32 can process it in under 100μs. The FIFO (128 bytes on ESP32) is more than sufficient to buffer messages while the CPU is busy.

## Baud Rate Decision: 115200

We tested three baud rates: 9600, 115200, and 921600. At 9600 baud, communication was reliable but slow: sending a 50-byte sensor reading took 5ms, which limited our sensor update rate to 200Hz. At 115200 baud, the same message took 0.4ms, supporting 2500 messages per second — more than enough for our 100Hz sensor update requirement. At 921600 baud, we measured occasional packet errors (approximately 1 in 10000 messages had a framing error or parity mismatch), likely due to the long cable (30cm) without proper termination.

We settled on 115200 baud as the best balance of speed and reliability. The 0.4ms transmission time for a typical command is negligible compared to the sensor reading time (30ms for VL53L0X). Even if we doubled the message size to include full sensor telemetry, 115200 baud would still handle 500 messages per second.

## Using UART1 Instead of UART0

An important discovery was that the ESP32's UART0 is connected to the USB-to-serial converter chip (CP2102 or CH340) on most development boards. When we flash firmware or monitor debug output via USB, we are using UART0. If UART0 is also connected to the Pi, the flashing tool would conflict with the Pi's communication. We needed to use UART1 for the robot communication channel.

On the ESP32-S3, UART1 can be mapped to any GPIO pins. We chose GPIO 16 (TX) and GPIO 17 (RX) for the Pi connection. This leaves UART0 free for debugging via USB, allowing us to monitor the ESP32's debug output while the robot is running — a huge advantage for development. During the competition, we will remove the USB cable, and UART0 will be unused.

## Learned: ESP32 Bootloader Prints Garbage

The most important lesson from v1.5 is that the ESP32's ROM bootloader is not silent. It prints diagnostic information over UART0 at boot, regardless of whether the user's application uses UART. If UART1 is connected to the Pi, this is not an issue (the bootloader only uses UART0). However, if someone were to accidentally connect UART0 to the Pi, they would see garbage bytes at startup and wonder why their protocol is broken.

We now include a comment at the top of the UART configuration code reminding us to use UART1 for robot communication and to flush the buffer at startup. This is documented in our hardware pinout reference.

## UART Protocol Design

With the ping-pong test passing, we defined the full UART protocol for the robot:

- All messages are ASCII text, newline-terminated (\n)
- Maximum message length: 128 bytes (fits in the ESP32's hardware FIFO)
- Pi sends commands: "MOTOR left=0.5 right=0.8\n", "SERVO angle=-15\n", "SENSOR\n"
- ESP32 responds: "OK\n" for commands, "SENSOR ax=0.12 ay=0.01 az=1.00 gx=0.5 gy=0.1 gz=-0.2\n" for sensor requests
- Error responses: "ERR unknown command\n" or "ERR motor timeout\n"

The text-based protocol was a deliberate choice. Binary protocols are more efficient but opaque: when debugging, you cannot see what is being sent without a protocol analyzer. With ASCII, we can connect any serial terminal (screen, minicom, PuTTY) and watch the conversation in real time. The slight overhead of ASCII encoding (e.g., "255" vs. 0xFF) is negligible at our message sizes.
