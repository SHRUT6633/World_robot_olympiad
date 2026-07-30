/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/failsafe.c
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: Emergency stop and safety subsystem
 * ===========================================================================
 */

#include "failsafe.h"
#include "l298n.h"
#include "servo_pwm.h"
#include "esp_log.h"

/* Tag used for ESP_LOG* messages to identify this module. */
static const char *TAG = "FAILSAFE";

/* failsafe_init
 * Placeholder initialisation. The actual hardware dependencies
 * (motor driver, servo) are initialised separately through their
 * own init functions.
 *
 * In a production system this might configure a hardware latch
 * (e.g. a relay or MOSFET enable line) that can be used to
 * physically disconnect motor power.
 */
void failsafe_init(void) {
    ESP_LOGI(TAG, "Failsafe initialized");
}

/* failsafe_engage
 * Emergency stop routine.
 *
 * Called when a critical condition is detected:
 *   - Received command 0xFF (emergency stop).
 *   - Communication timeout expired.
 *   - Watchdog timer fires.
 *   - Self-test failure.
 *
 * Implementation:
 *   1. Stops the L298N motor by setting speed to 0 %.
 *      This sets PWM duty to 0 and IN1/IN2 both low → motor coasts.
 *   2. Returns the servo to its neutral (0°) position.
 *      The servos will hold this position as long as power is applied.
 *
 * If the motor driver uses a dedicated enable line (e.g. L298N ENA),
 * setting speed to 0 effectively disables the output. For extra
 * safety, IN1 and IN2 could be set to LOW together (coast mode)
 * or HIGH together (brake mode) depending on mechanical requirements.
 *
 * Note: This does NOT cut main power. In a safety-critical system,
 * a redundant hardware kill-switch should be present.
 */
void failsafe_engage(void) {
    ESP_LOGW(TAG, "FAILSAFE ENGAGED - stopping all motors");
    l298n_set_motor(0, true);   /* 0 % speed, direction arbitrary */
    servo_set_angle(0);          /* centre the servo */
}
