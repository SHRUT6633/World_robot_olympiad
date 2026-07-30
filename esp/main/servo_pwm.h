/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/servo_pwm.h
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: Header for servo PWM control
 * ===========================================================================
 */

#ifndef SERVO_PWM_H
#define SERVO_PWM_H

/* servo_pwm_init
 * Initialises the LEDC timer/channel for servo PWM on the configured
 * GPIO pin. Sets a 50 Hz base frequency with 12-bit resolution.
 * Must be called before servo_set_angle().
 */
void servo_pwm_init(void);

/* servo_set_angle
 * Commands the servo to a given angle in degrees.
 * angle_deg : desired angle, clamped to ±SERVO_MAX_ANGLE (±30°).
 * Internally maps the angle to a pulse width between 500 µs and
 * 2500 µs, centred at 1500 µs. The pulse width is then converted
 * to a 12-bit LEDC duty value for the 50 Hz PWM signal.
 *
 * Standard RC servo timing:
 *   500  µs → one extreme
 *   1500 µs → centre (0°)
 *   2500 µs → opposite extreme
 *
 * The maximum angle (±30°) is a safety limit; modifying
 * SERVO_MAX_ANGLE or the pulse-width constants changes the
 * servo's range of motion.
 */
void servo_set_angle(float angle_deg);

#endif
