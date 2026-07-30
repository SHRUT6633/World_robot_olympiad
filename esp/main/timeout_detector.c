/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/timeout_detector.c
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: Communications timeout detection
 * ===========================================================================
 */

#include "timeout_detector.h"
#include "esp_timer.h"

/* Internal state of the timeout detector.
 *
 * s_timeout_us    : the configured timeout period in microseconds.
 *                   Default 500000 µs (0.5 s). Set by
 *                   timeout_detector_init().
 *
 * s_last_reset_us : the value of esp_timer_get_time() at the last
 *                   reset (or initialisation). This is the reference
 *                   point for trigger detection.
 */
static uint64_t s_timeout_us = 500000;
static uint64_t s_last_reset_us = 0;

/* timeout_detector_init
 * Stores the desired timeout and records the current time as the
 * base reference.
 *
 * timeout_us : the maximum allowed interval between resets, in µs.
 *
 * If the timeout is set very short (e.g., < 10 ms), the system
 * must call reset() frequently enough to avoid false triggers.
 * If set too long, a lost connection may not be detected in time
 * to prevent a collision.
 */
void timeout_detector_init(uint64_t timeout_us) {
    s_timeout_us = timeout_us;
    s_last_reset_us = esp_timer_get_time();
}

/* timeout_detector_reset
 * Updates the last-reset timestamp to "now".
 * Should be called every time a valid packet is received and
 * processed.
 */
void timeout_detector_reset(void) {
    s_last_reset_us = esp_timer_get_time();
}

/* timeout_detector_triggered
 * Compares the elapsed time since the last reset against the timeout.
 *
 *   triggered = (now - last_reset) > timeout
 *
 * If the hardware timer overflows (approx. every 584 years with
 * a 64-bit µs counter), the subtraction correctly wraps because
 * unsigned arithmetic is used. This is safe.
 *
 * Returns true if the timeout has expired.
 */
bool timeout_detector_triggered(void) {
    uint64_t now = esp_timer_get_time();
    return (now - s_last_reset_us) > s_timeout_us;
}
