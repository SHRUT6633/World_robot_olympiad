/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/command_validator.h
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: Header for command validation
 * ===========================================================================
 */

#ifndef COMMAND_VALIDATOR_H
#define COMMAND_VALIDATOR_H

#include <stdint.h>
#include <stdbool.h>

/* Enumerated result codes from validate_command().
 *
 * CMD_VALID          : the command type, length, and payload are acceptable.
 * CMD_INVALID_TYPE   : the message type byte is not recognised.
 * CMD_INVALID_LENGTH : the payload length does not match expectations.
 * CMD_INVALID_RANGE  : the payload value is outside the allowed range
 *                      (currently only checked for motor commands).
 */
typedef enum {
    CMD_VALID = 0,
    CMD_INVALID_TYPE,
    CMD_INVALID_LENGTH,
    CMD_INVALID_RANGE,
} cmd_validation_t;

/* validate_command
 * Checks the semantic validity of a decoded command from the packet.
 *
 * msg_type : command identifier byte (e.g. 0x01 = motor, 0x02 = servo).
 * length   : length of the payload field in bytes.
 * payload  : pointer to the payload bytes.
 *
 * Returns a cmd_validation_t code indicating success or the reason
 * for rejection.
 *
 * Known command types:
 *   0x01 – Motor command      (length = 1, payload[0] = speed)
 *   0x02 – Servo command      (length = 4, float angle)
 *   0x03 – Steering command   (length = 5, combined motor+servo)
 *   0x04 – Status request     (length = 0, no payload)
 *   0xFF – Emergency stop     (length = 0, no payload)
 *
 * Adding a new command type requires:
 *   1. Adding a case to the switch.
 *   2. Updating MAX_MOTOR_SPEED or MAX_SERVO_ANGLE as needed.
 *   3. Updating the remote controller to send the new type.
 */
cmd_validation_t validate_command(uint8_t msg_type, uint8_t length,
                                  const uint8_t *payload);

#endif
