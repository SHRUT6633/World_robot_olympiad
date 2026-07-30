#include "watchdog.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

/* Tag used for ESP_LOG* messages to identify this module. */
static const char *TAG = "WDT";

/* watchdog_init
 * Configures and starts the ESP32 Task Watchdog Timer (TWDT).
 *
 * Configuration:
 *   timeout_ms    = 3000  : 3 seconds without a feed triggers the WDT.
 *   idle_core_mask = 0    : do not watch idle tasks on any core.
 *   trigger_panic = true  : on timeout, trigger a panic (panic handler
 *                           typically prints a backtrace and reboots).
 *
 * If the TWDT was already initialised (ESP_ERR_INVALID_STATE), the
 * function reconfigures it with the new settings. This handles the
 * case where another component (e.g., the bootloader or ESP-IDF) has
 * already set up a WDT.
 *
 * After init, the calling task is added to the watch list via
 * esp_task_wdt_add(NULL), where NULL means the current task.
 *
 * WARNING: A 3-second timeout is generous but should be tuned based
 * on the worst-case execution time of the main loop.
 */
void watchdog_init(void) {
    esp_task_wdt_config_t config = {
        .timeout_ms = 3000,          /* 3 s timeout */
        .idle_core_mask = 0,         /* do not watch idle tasks */
        .trigger_panic = true,       /* reboot on timeout */
    };

    esp_err_t err = esp_task_wdt_init(&config);
    if (err == ESP_ERR_INVALID_STATE) {
        /* TWDT was already initialised; reconfigure. */
        ESP_LOGW(TAG, "TWDT already initialized, reconfiguring...");
        ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&config));
    } else {
        ESP_ERROR_CHECK(err);
    }

    /* Add the current task to the watchdog subscription. */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_LOGI(TAG, "Watchdog configured (3s timeout)");
}

/* watchdog_feed
 * Resets the watchdog timer for the current task.
 * Called periodically from the main loop to indicate that the
 * task is still alive and processing normally.
 *
 * If the main loop blocks for more than 3 seconds (e.g., on a
 * blocking UART read without a timeout), the watchdog will fire
 * and the system will reboot. Ensure that any blocking operations
 * either have a shorter timeout or are interleaved with feed calls.
 */
void watchdog_feed(void) {
    esp_task_wdt_reset();
}
