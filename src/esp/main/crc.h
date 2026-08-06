/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/crc.h
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: Header for CRC-16 computation
 * ===========================================================================
 */

#ifndef CRC_H
#define CRC_H

#include <stdint.h>

/* crc16
 * Computes a 16-bit CRC over a byte buffer using the CRC-16-IBM
 * (also known as CRC-16-ANSI) polynomial 0x8005.
 *
 * data : pointer to the input byte array.
 * len  : number of bytes to process.
 *
 * Returns the 16-bit CRC value.
 *
 * Algorithm:
 *   - Initial remainder = 0xFFFF.
 *   - For each byte: XOR with top byte, then process 8 bits
 *     with the polynomial 0x8005 (x^16 + x^15 + x^2 + 1).
 *   - Final remainder is returned (no post-inversion).
 *
 * This CRC is used in the packet protocol to detect transmission
 * errors. Both sender and receiver must agree on the polynomial
 * and initial value.
 */
uint16_t crc16(const uint8_t *data, int len);

#endif
