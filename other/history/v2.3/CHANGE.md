# v2.3 — Odometry from Wheel Encoders

## What changed

The robot now knows how far it has traveled. I added two magnetic rotary encoders (AS5600) to the rear wheel hubs, one per wheel. Each encoder outputs 12-bit absolute position over I2C. I wrote `encoder_driver.c` on the ESP32 to read both encoders at 100 Hz and compute incremental tick counts, then `odometry.py` on the Pi to read the tick counts over UART and estimate distance traveled.

The AS5600 is a hall-effect magnetic encoder that sits under a diametrically magnetized magnet attached to the wheel shaft. It outputs 0-4095 counts per revolution. With 65 mm diameter wheels, each count represents `(pi * 65) / 4095 = 0.0499 mm`. At 100 Hz read rate, we can measure speed up to about 2 m/s before aliasing (4096 counts/rev * 100 Hz * 0.0499 mm/count = 20.4 m/s equivalent, so plenty of headroom).

The data flow is:
1. ESP32 reads both encoders via I2C at 10 ms intervals.
2. ESP32 computes delta ticks since last reading for each wheel.
3. Pi sends `{"cmd": "poll_odometry"}` or the ESP32 automatically sends odometry at 10 Hz.
4. `odometry.py` accumulates the ticks and computes: `distance = (left_delta + right_delta) / 2 * ticks_to_mm`, `heading_delta = (right_delta - left_delta) / track_width * ticks_to_mm` (in radians).

## Why it changed

Without odometry, the robot is blind to its own motion. The PID controller in v2.4 needs to know if the robot is actually moving straight. The trajectory planner in v2.5 needs to know how far the robot has traveled to time the next command. The competition requires the robot to follow a line, but if the line is temporarily lost (e.g., a gap in the tape), odometry can dead-reckon through the gap.

## Errors encountered

The first implementation used GPIO interrupts on the ESP32 to count encoder pulses. The AS5600 has a PWM output that pulses once per count. I connected this to GPIO 4 (left) and GPIO 5 (right) and set up rising-edge interrupts:

```c
gpio_isr_handler_add(GPIO_NUM_4, encoder_isr_left, NULL);
gpio_isr_handler_add(GPIO_NUM_5, encoder_isr_right, NULL);
```

This worked perfectly at low speed (below 0.5 m/s). At higher speeds, the interrupt handler started missing pulses. I verified this by comparing the interrupt-based count against the I2C-based absolute position read every second. At 1 m/s, the interrupt count was about 15% lower than the actual position change.

The problem is that the ESP32's GPIO interrupt latency is not deterministic. The interrupt service routine runs at the highest priority, but if two interrupts arrive close together (e.g., left and right encoders both triggering within a few microseconds), the second one can be dropped while the first one is being processed. At 1 m/s, each encoder generates about 3120 pulses per second (1.0 / (pi * 0.065) * 4095 / 2), which means one pulse every 320 microseconds on average. The ISR takes about 5-10 microseconds to run. That should be fine. But the two wheels were triggering simultaneously on rough surfaces—the bumps made both wheels vibrate and generate simultaneous pulses. The interrupt handler for the left wheel was still running when the right wheel's interrupt arrived, and the right wheel's interrupt was marked as pending but then lost when the left handler cleared the interrupt flag register.

I found this by reading the ESP32's interrupt status register after each ISR call:

```c
// Diagnostic: check if we missed an interrupt
uint32_t status = GPIO.interrupt_status0.val;
if (status & (1 << 5)) {
    // Right wheel interrupt is pending but not being serviced!
    missed_count++;
}
```

At 1 m/s, `missed_count` was about 450 per second.

The fix was to switch from GPIO interrupts to the ESP32's PCNT (Pulse Counter) peripheral. The PCNT module is a hardware counter that counts pulses on a GPIO pin independently of the CPU. It can count up to 65535 pulses before overflowing, and we can read the count at any time without worrying about missed interrupts.

```c
pcnt_config_t pcnt_cfg = {
    .pulse_gpio_num = GPIO_NUM_4,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
};
pcnt_unit_config(&pcnt_cfg);
```

With PCNT, the missed pulse problem disappeared entirely. At 1.5 m/s, the PCNT count matched the I2C absolute position to within 0.1% (the remaining error is quantization noise from the AS5600's 12-bit resolution).

## Alternative approaches considered

1. **Quadrature encoders**: Use a proper quadrature encoder with two channels per wheel (A and B) to detect direction. The AS5600 is a single-channel absolute encoder, so it can't detect reverse direction without reading the full 12-bit value over I2C. For the basic driving phase, we only drive forward, so direction isn't needed. But it will be needed for v2.6 (stop and reverse).

2. **Optical encoders**: Slotted optical encoders with an LED and phototransistor are cheaper than magnetic encoders. But they're susceptible to dust and misalignment. The AS5600 is sealed and more reliable.

3. **External interrupt controller**: Add an I/O expander with interrupt capability (e.g., MCP23017) to handle the encoder counting. This adds complexity and cost.

I stuck with the PCNT approach because it's a hardware feature we're already paying for (it's built into the ESP32) and it solves the problem perfectly.

## Reasoning

Odometer accuracy at this stage doesn't need to be perfect. The AS5600's 12-bit resolution gives us about 0.05 mm per count, but wheel slip, tire pressure, and surface variations mean the actual accuracy is probably ±5% over a 10 m run. For now, that's good enough for the PID controller and trajectory planner to work with. We can calibrate the odometry later using a known-distance run.

The PCNT-based approach also frees up CPU time. The interrupt-based approach was consuming about 3% of CPU time at 1 m/s (3120 ISR calls/sec * 10 µs = 31 ms/sec). The PCNT approach consumes about 0.1% CPU time (just polling the counter register at 100 Hz). This leaves more CPU for the PID controller, UART communication, and other tasks.

One issue I noticed: the AS5600 reading over I2C takes about 2 ms per read (two bytes at 400 kHz I2C). Reading both encoders sequentially takes 4 ms, which is 40% of our 10 ms control loop. I optimized the I2C read to use the ESP32's I2C master with a 100-byte buffer, reading both encoders in a single transaction by sending a repeated start condition. This cut the read time to 2.2 ms total.
