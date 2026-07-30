#include "crc.h"

/* CRC-16-IBM (CRC-16-ANSI) polynomial: x^16 + x^15 + x^2 + 1.
 * In binary (MSB-first): 1100 0000 0000 0101 → 0x8005.
 *
 * This is the polynomial used by MODBUS, USB, and many other
 * protocols. It provides good error detection for packets of
 * moderate length.
 */
#define CRC_POLY 0x8005

/* crc16
 * Computes CRC-16-IBM over the given data buffer.
 *
 * Implementation notes:
 *   - Uses the "naive" bit-by-bit method (not table-driven).
 *     This is slower but uses very little code space.
 *   - Shift direction: MSB-first (big-endian).
 *   - No final XOR; the raw remainder is returned.
 *
 * If throughput is critical, replace this with a 256-entry
 * lookup-table version. Both sides must use the same algorithm.
 */
uint16_t crc16(const uint8_t *data, int len) {
    /* Initial CRC value (0xFFFF per IBM spec). */
    uint16_t crc = 0xFFFF;

    /* Process each byte of the input. */
    for (int i = 0; i < len; i++) {
        /* XOR the incoming byte into the upper byte of the CRC. */
        crc ^= (uint16_t)data[i] << 8;

        /* Process eight bits, one at a time. */
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                /* MSB is set: shift and XOR with polynomial. */
                crc = (crc << 1) ^ CRC_POLY;
            } else {
                /* MSB is clear: just shift left. */
                crc <<= 1;
            }
        }
    }

    return crc;
}
