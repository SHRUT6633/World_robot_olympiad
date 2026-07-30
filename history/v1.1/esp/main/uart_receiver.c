#include "uart_receiver.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "UART_RX";
static int s_uart_port = UART_NUM_1;

void uart_receiver_init(void) {
    ESP_LOGI(TAG, "UART receiver ready");
}

int uart_receive_packet(uint8_t *buf, int max_len) {
    int len = uart_read_bytes(s_uart_port, buf, max_len, pdMS_TO_TICKS(10));
    return len;
}
