#ifndef UART_RECEIVER_H
#define UART_RECEIVER_H

#include <stdint.h>

/* uart_receiver_init
 * Initialises the UART receiver module.
 * Currently a no-op placeholder; actual UART driver configuration
 * (baud rate, pins, parity, etc.) is expected to be added here
 * or performed externally.
 */
void uart_receiver_init(void);

/* uart_receive_packet
 * Reads bytes from the UART receive buffer into a caller-supplied
 * buffer. Non-blocking with a short timeout (10 ms).
 *
 * buf     : destination buffer for received bytes.
 * max_len : maximum number of bytes to read.
 *
 * Returns the number of bytes actually read, or 0 if no data was
 * available within the timeout period. Returns -1 on error.
 *
 * The caller should validate the packet (header, footer, CRC)
 * using validate_packet() from packet_validator.h after a
 * successful read.
 */
int uart_receive_packet(uint8_t *buf, int max_len);

#endif
