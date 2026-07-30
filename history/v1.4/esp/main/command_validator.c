#include "command_validator.h"

/* Maximum allowed servo angle in degrees (±30).
 * Exceeding this could damage the servo or the mechanism.
 * This should match SERVO_MAX_ANGLE in servo_pwm.c.
 */
#define MAX_SERVO_ANGLE 30.0f

/* Maximum allowed motor speed percentage (0..100).
 * 100 corresponds to 100 % PWM duty on the L298N ENA pin.
 */
#define MAX_MOTOR_SPEED 100

/* validate_command
 * Validates a command based on its type byte, payload length, and
 * optionally the value range of the payload.
 *
 * The length check ensures that the packet payload matches exactly
 * what the downstream handler expects. A mismatch could cause buffer
 * over-reads or undefined behaviour.
 *
 * Current commands:
 *   0x01 (Motor)     : 1-byte payload = signed speed.
 *                      Validated to be ≤ MAX_MOTOR_SPEED.
 *   0x02 (Servo)     : 4-byte payload = float32 angle.
 *                      Range check must be done by servo_set_angle().
 *   0x03 (Steering)  : 5-byte payload = combined command.
 *                      Format TBD; length check only.
 *   0x04 (Status)    : 0-byte payload. Just request status.
 *   0xFF (E-stop)    : 0-byte payload. Triggers failsafe.
 *
 * Returns CMD_VALID if the command passes all checks, or an
 * appropriate error code otherwise.
 */
cmd_validation_t validate_command(uint8_t msg_type, uint8_t length,
                                  const uint8_t *payload) {
    switch (msg_type) {
        case 0x01: /* Motor command */
            /* Expect exactly one payload byte (speed). */
            if (length != 1) return CMD_INVALID_LENGTH;
            /* Speed value must not exceed 100 %. */
            if (payload[0] > MAX_MOTOR_SPEED) return CMD_INVALID_RANGE;
            return CMD_VALID;

        case 0x02: /* Servo command */
            /* Expect exactly 4 bytes (IEEE 754 float). */
            if (length != 4) return CMD_INVALID_LENGTH;
            return CMD_VALID;

        case 0x03: /* Steering command */
            /* Expect exactly 5 bytes (motor + servo combined). */
            if (length != 5) return CMD_INVALID_LENGTH;
            return CMD_VALID;

        case 0x04: /* Status request */
            /* No payload expected. */
            if (length != 0) return CMD_INVALID_LENGTH;
            return CMD_VALID;

        case 0xFF: /* Emergency stop */
            /* No payload expected; takes effect immediately. */
            return CMD_VALID;

        default:
            /* Unknown message type. */
            return CMD_INVALID_TYPE;
    }
}
