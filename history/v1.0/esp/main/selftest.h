#ifndef SELFTEST_H
#define SELFTEST_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool uart_ok;
    bool servo_pwm_ok;
    bool motor_pwm_ok;
    bool tb6612fng_ok;
    bool watchdog_ok;
    uint32_t test_duration_ms;
} esp_selftest_result_t;

void esp_selftest_init(void);
void esp_selftest_run(esp_selftest_result_t *result);
bool esp_selftest_all_passed(const esp_selftest_result_t *result);

#endif
