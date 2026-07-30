#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    printf("ESP32-S3 Firmware v1.0\n");
    printf("Initializing...\n");
    // TODO: add motor driver, servo, UART
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Ready.\n");
}
