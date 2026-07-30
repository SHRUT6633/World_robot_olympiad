#ifndef PACKET_VALIDATOR_H
#define PACKET_VALIDATOR_H

#include <stdint.h>
#include <stdbool.h>

/* validate_packet
 * Checks whether a received byte buffer conforms to the expected
 * packet format and passes CRC-16 verification.
 *
 * data : pointer to the received byte buffer.
 * len  : total number of bytes in the buffer.
 *
 * Expected format:
 *   Byte 0         : header (0xA5)
 *   Bytes 1..len-4 : payload + type + length (covered by CRC)
 *   Bytes len-3    : CRC low byte
 *   Bytes len-2    : CRC high byte
 *   Byte len-1     : footer (0x5A)
 *
 * Returns true if the packet is valid, false otherwise.
 * Validation steps:
 *   1. Minimum length check (≥ 8 bytes).
 *   2. Header byte must equal PACKET_HEADER.
 *   3. Footer byte must equal PACKET_FOOTER.
 *   4. CRC-16 computed over bytes [0 .. len-4] must match the
 *      two CRC bytes stored at positions len-3 and len-2.
 *
 * The header (0xA5) and footer (0x5A) provide basic frame
 * synchronisation. The CRC covers everything except the CRC
 * and footer so that the CRC itself is not self-verified.
 */
bool validate_packet(const uint8_t *data, int len);

#endif
