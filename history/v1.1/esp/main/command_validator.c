#include "command_validator.h"

#define MAX_SERVO_ANGLE 30.0f
#define MAX_MOTOR_SPEED 100

cmd_validation_t validate_command(uint8_t msg_type, uint8_t length, const uint8_t *payload) {
    switch (msg_type) {
        case 0x01: // Motor command
            if (length != 1) return CMD_INVALID_LENGTH;
            if (payload[0] > MAX_MOTOR_SPEED) return CMD_INVALID_RANGE;
            return CMD_VALID;

        case 0x02: // Servo command
            if (length != 4) return CMD_INVALID_LENGTH;
            return CMD_VALID;

        case 0x03: // Steering command
            if (length != 5) return CMD_INVALID_LENGTH;
            return CMD_VALID;

        case 0x04: // Status request
            if (length != 0) return CMD_INVALID_LENGTH;
            return CMD_VALID;

        case 0xFF: // Emergency stop
            return CMD_VALID;

        default:
            return CMD_INVALID_TYPE;
    }
}
