/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/l298n.h
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: Header for L298N motor driver
 * ===========================================================================
 */

#ifndef L298N_H
#define L298N_H

#include <stdint.h>
#include <stdbool.h>

/* l298n_init
 * Initialises the L298N dual H-bridge motor driver.
 * Configures IN1 and IN2 GPIOs as push-pull outputs and sets up LEDC
 * PWM timer/channel for the ENA (enable/speed) pin.
 * Must be called once before l298n_set_motor().
 */
void l298n_init(void);

/* l298n_set_motor
 * Sets the motor speed and direction.
 * speed_pct : desired speed as a signed percentage (-100 to 100).
 *             Negative values are treated as reverse (forward=false).
 * forward   : true  = IN1 high, IN2 low (forward rotation).
 *             false = IN1 low, IN2 high (reverse rotation).
 * The function clamps speed_pct to [0, 100] and maps it to a 10-bit
 * PWM duty cycle (0..1023) on the LEDC channel tied to ENA.
 * Engaging the driver requires that the L298N's ENA pin is jumpered
 * or driven by the PWM signal (GPIO 11) configured in l298n_init().
 */
void l298n_set_motor(int speed_pct, bool forward);

#endif
