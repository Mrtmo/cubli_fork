#ifndef _MOTOR2204_H_
#define _MOTOR2204_H_

#include <stdint.h>

extern void motor_pwm_init(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3, uint8_t pwm_enable);
extern float velocity_openloop(float target_velocity);

#endif