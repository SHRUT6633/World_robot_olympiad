#ifndef PACKET_VALIDATOR_H
#define PACKET_VALIDATOR_H

#include <stdint.h>
#include <stdbool.h>

bool validate_packet(const uint8_t *data, int len);

#endif
