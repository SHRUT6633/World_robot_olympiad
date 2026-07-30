#ifndef UART_RECEIVER_H
#define UART_RECEIVER_H

#include <stdint.h>

void uart_receiver_init(void);
int uart_receive_packet(uint8_t *buf, int max_len);

#endif
