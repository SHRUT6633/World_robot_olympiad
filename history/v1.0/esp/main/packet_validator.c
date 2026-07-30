#include "packet_validator.h"
#include "crc.h"

#define PACKET_HEADER 0xA5
#define PACKET_FOOTER 0x5A

bool validate_packet(const uint8_t *data, int len) {
    if (len < 8) return false;
    if (data[0] != PACKET_HEADER) return false;
    if (data[len - 1] != PACKET_FOOTER) return false;
    uint16_t pkt_crc = (uint16_t)data[len - 3] | ((uint16_t)data[len - 2] << 8);
    uint16_t calc = crc16(data, len - 3);
    return calc == pkt_crc;
}
