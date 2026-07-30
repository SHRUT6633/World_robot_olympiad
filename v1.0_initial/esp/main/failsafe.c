#include "failsafe.h"
#include "motor_pwm.h"
#include "servo_pwm.h"
#include "esp_log.h"

static const char *TAG = "FAILSAFE";

void failsafe_init(void) {
    ESP_LOGI(TAG, "Failsafe initialized");
}

void failsafe_engage(void) {
    ESP_LOGW(TAG, "FAILSAFE ENGAGED - stopping all motors");
    motor_set_speed(0);
    servo_set_angle(0);
}
