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
    esp_err_t err = esp_task_wdt_init(&config);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "TWDT already initialized, reconfiguring...");
        ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&config));
    } else {
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_LOGI(TAG, "Watchdog configured (3s timeout)");
}

void watchdog_feed(void) {
    esp_task_wdt_reset();
}
