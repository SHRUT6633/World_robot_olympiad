/**
 * ===========================================================================
 * WRO 2026 — 4WS AWD Autonomous Robot
 * File: esp/main/uart_receiver.c
 * Rev:  v9.9  |  Status: RELEASED
 * ---------------------------------------------------------------------------
 * PURPOSE: UART packet receiver
 * ===========================================================================
 */

#include "uart_receiver.h"
#include "driver/uart.h"
#include "esp_log.h"

/* Tag used for ESP_LOG* messages to identify this module. */
static const char *TAG = "UART_RX";

/* UART port number used for receiving packets from the remote
 * controller / Raspberry Pi. UART_NUM_0 corresponds to the
 * ESP32-S3's first UART (GPIO43/44), wired to the board's USB-UART
 * bridge (CH343) — the protocol now travels over the USB cable.
 */
static int s_uart_port = UART_NUM_0;

/* uart_receiver_init
 * Logs that the UART receiver is ready.
 *
 * Note: Actual UART parameter configuration (baud rate 115200,
 * data bits 8, parity none, stop bits 1, flow control none, and
 * pin mapping) is not performed here. The upstream initialisation
 * code (e.g. main.c) is expected to call uart_param_config() and
 * uart_driver_install() before invoking this function.
 *
 * If UART configuration is added here, ensure the baud rate matches
 * the transmitter (typically 115200 or 9600 bps).
 */
void uart_receiver_init(void) {
    ESP_LOGI(TAG, "UART receiver ready");
}

/* uart_receive_packet
 * Attempts to read up to max_len bytes from the UART receive queue
 * with a 10 ms blocking timeout.
 *
 * Returns the number of bytes actually received, 0 if the timeout
 * elapsed with no data, or a negative error code from the UART
 * driver.
 *
 * The 10 ms timeout keeps the calling task responsive; a longer
 * timeout would reduce CPU usage but increase command latency.
 * A shorter timeout would increase responsiveness at the cost of
 * more frequent polling.
 */
int uart_receive_packet(uint8_t *buf, int max_len) {
    int len = uart_read_bytes(s_uart_port, buf, max_len, pdMS_TO_TICKS(10));
    return len;
}
