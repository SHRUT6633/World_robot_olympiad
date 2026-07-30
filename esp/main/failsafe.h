/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/failsafe.h
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: Header for failsafe subsystem
 * ===========================================================================
 */

#ifndef FAILSAFE_H
#define FAILSAFE_H

/* failsafe_init
 * Initialises the failsafe subsystem.
 * Currently a no-op; logs that the failsafe is ready.
 */
void failsafe_init(void);

/* failsafe_engage
 * Immediately stops all actuators as a safety response.
 *
 * Actions:
 *   - Sets motor speed to 0 (via l298n_set_motor).
 *   - Returns servo to centre/neutral angle (via servo_set_angle).
 *
 * This is called when:
 *   - An emergency stop command (0xFF) is received.
 *   - The timeout_detector fires (communication lost).
 *   - Any critical error is detected.
 *
 * After engage(), the robot is safe but may need re-initialisation
 * to resume normal operation.
 */
void failsafe_engage(void);

#endif
