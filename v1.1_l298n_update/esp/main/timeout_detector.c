#include "timeout_detector.h"
#include "esp_timer.h"

static uint64_t s_timeout_us = 500000;
static uint64_t s_last_reset_us = 0;

void timeout_detector_init(uint64_t timeout_us) {
    s_timeout_us = timeout_us;
    s_last_reset_us = esp_timer_get_time();
}

void timeout_detector_reset(void) {
    s_last_reset_us = esp_timer_get_time();
}

bool timeout_detector_triggered(void) {
    uint64_t now = esp_timer_get_time();
    return (now - s_last_reset_us) > s_timeout_us;
}
