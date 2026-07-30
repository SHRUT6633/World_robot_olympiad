#ifndef COMMAND_VALIDATOR_H
#define COMMAND_VALIDATOR_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CMD_VALID = 0,
    CMD_INVALID_TYPE,
    CMD_INVALID_LENGTH,
    CMD_INVALID_RANGE,
} cmd_validation_t;

cmd_validation_t validate_command(uint8_t msg_type, uint8_t length, const uint8_t *payload);

#endif
