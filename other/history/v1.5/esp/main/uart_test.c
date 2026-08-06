#include "driver/uart.h"
#include "string.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE 128

void uart_test(void) {
    uart_config_t cfg = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
    uart_param_config(UART_NUM, &cfg);
    uart_set_pin(UART_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);
    uart_flush(UART_NUM);  // Crucial: clear bootloader garbage
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t buf[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, buf, BUF_SIZE-1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            buf[len] = 0;
            if (strcmp((char*)buf, "ping\n") == 0) {
                uart_write_bytes(UART_NUM, "pong\n", 5);
                printf("UART: ping -> pong\n");
            }
        }
    }
}
