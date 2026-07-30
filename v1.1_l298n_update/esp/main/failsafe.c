#include "failsafe.h"
#include "l298n.h"
#include "servo_pwm.h"
#include "esp_log.h"

static const char *TAG = "FAILSAFE";

void failsafe_init(void) {
    ESP_LOGI(TAG, "Failsafe initialized");
}

void failsafe_engage(void) {
    ESP_LOGW(TAG, "FAILSAFE ENGAGED - stopping all motors");
    l298n_set_motor(0, true);
    servo_set_angle(0);
}
