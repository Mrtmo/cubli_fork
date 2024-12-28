#ifndef _MOTOR2204_H_
#define _MOTOR2204_H_

#include <stdint.h>

extern void motor_pwm_init(gpio_num_t pwm1, gpio_num_t pwm2, gpio_num_t pwm3, gpio_num_t pwm_enable);
extern void motor_disable(void);
extern void motor_enable(void);
extern float velocity_openloop(float target_velocity);

#endif