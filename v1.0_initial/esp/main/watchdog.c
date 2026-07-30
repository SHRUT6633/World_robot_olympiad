#include "watchdog.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

static const char *TAG = "WDT";

void watchdog_init(void) {
    esp_task_wdt_config_t config = {
        .timeout_ms = 3000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&config));
    ESP_LOGI(TAG, "Watchdog initialized (3s timeout)");
}

void watchdog_feed(void) {
    esp_task_wdt_reset();
}
