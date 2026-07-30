#ifndef SELFTEST_H
#define SELFTEST_H

#include <stdint.h>
#include <stdbool.h>

/* Structure holding the results of all self-test checks.
 *
 * Fields:
 *   uart_ok        : test_led result (reuses field; was UART test).
 *                    Currently drives the onboard LED (GPIO 2).
 *   servo_pwm_ok   : LEDC channel 0 (servo) PWM write succeeded.
 *   motor_pwm_ok   : LEDC channel 1 (motor) PWM write succeeded.
 *   l298n_ok       : IN1/IN2 GPIO toggle succeeded.
 *   watchdog_ok    : watchdog module self-check (currently
 *                    always passes – a placeholder).
 *   test_duration_ms: elapsed wall-clock time of the test run.
 */
typedef struct {
    bool uart_ok;             /* LED test (repurposed) result */
    bool servo_pwm_ok;        /* Servo PWM channel test result */
    bool motor_pwm_ok;        /* Motor PWM channel test result */
    bool l298n_ok;            /* L298N direction pin test result */
    bool watchdog_ok;         /* Watchdog self-check result */
    uint32_t test_duration_ms;/* Total test duration in ms */
} esp_selftest_result_t;

/* esp_selftest_init
 * Prepares GPIOs and any other resources needed by the self-test.
 * Currently configures the onboard LED (GPIO 2) as an output.
 */
void esp_selftest_init(void);

/* esp_selftest_run
 * Executes all individual hardware self-tests and populates the
 * result structure. Tests are run sequentially.
 *
 * result : pointer to an esp_selftest_result_t that will be filled
 *          with pass/fail booleans and the total duration.
 *
 * If any test fails, the robot should not enter operational mode
 * until the fault is resolved.
 */
void esp_selftest_run(esp_selftest_result_t *result);

/* esp_selftest_all_passed
 * Convenience function that returns true if every test in the
 * result structure reported success.
 *
 * result : pointer to a populated esp_selftest_result_t.
 *
 * Returns true only when ALL of uart_ok, servo_pwm_ok,
 * motor_pwm_ok, l298n_ok, AND watchdog_ok are true.
 */
bool esp_selftest_all_passed(const esp_selftest_result_t *result);

#endif
