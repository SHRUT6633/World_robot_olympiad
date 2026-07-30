#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdint.h>

void motor_pwm_init(void);
void motor_set_speed(uint8_t speed_pct);

#endif
