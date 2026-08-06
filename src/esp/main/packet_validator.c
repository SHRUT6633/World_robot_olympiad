/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/packet_validator.c
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: Packet format validation and CRC verification
 * ===========================================================================
 */

#include "packet_validator.h"
#include "crc.h"

/* Frame delimiter bytes for packet synchronisation.
 *
 * PACKET_HEADER (0xA5) : marks the start of a packet.
 * PACKET_FOOTER (0x5A) : marks the end of a packet.
 *
 * These values are chosen to be distinct from common ASCII
 * characters to reduce false positives when scanning for
 * packet boundaries in a raw byte stream.
 */
#define PACKET_HEADER 0xA5
#define PACKET_FOOTER 0x5A

/* validate_packet
 * Performs structural and CRC-integrity checks on a received packet.
 *
 * Packet layout (big-endian CRC):
 *   [0]     = header  (0xA5)
 *   [1..k]  = payload (message type, length, data)
 *   [k+1]   = CRC low  byte
 *   [k+2]   = CRC high byte
 *   [k+3]   = footer  (0x5A)
 *
 * The CRC covers all bytes from index 0 up to (but not including)
 * the first CRC byte, i.e., data[0 .. len-3].
 *
 * Minimum length (8) accounts for:
 *   header(1) + type(1) + length(1) + payload(1) + CRC(2) + footer(1)
 *   = 7 bytes minimum payload, but with len=8 we have header+framing
 *   + at least 1 data byte.
 *
 * Returns true if all checks pass, false otherwise.
 */
bool validate_packet(const uint8_t *data, int len) {
    /* Reject packets too short to contain minimal framing. */
    if (len < 8) return false;

    /* Check start-of-frame marker. */
    if (data[0] != PACKET_HEADER) return false;

    /* Check end-of-frame marker. */
    if (data[len - 1] != PACKET_FOOTER) return false;

    /* Reconstruct the CRC from the two bytes in the packet.
     * CRC is stored in little-endian order (low byte first).
     */
    uint16_t pkt_crc = (uint16_t)data[len - 3] |
                       ((uint16_t)data[len - 2] << 8);

    /* Compute CRC over the entire packet up to (not including)
     * the CRC bytes. This includes the header, type, length,
     * and payload.
     */
    uint16_t calc = crc16(data, len - 3);

    /* CRC match → packet is valid. */
    return calc == pkt_crc;
}
