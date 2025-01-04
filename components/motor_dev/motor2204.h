#ifndef _MOTOR2204_H_
#define _MOTOR2204_H_

#include <stdint.h>
#include "as5600.h"


extern void motor_pwm_init(gpio_num_t pwm1, gpio_num_t pwm2, gpio_num_t pwm3, gpio_num_t pwm_enable);
extern void motor_disable(void);
extern void motor_enable(void);
extern void motor_link_as5600(i2c_as5600_handle_t as5600_handle);
extern float velocity_openloop(float target_velocity);
extern void weizhi_loop(float motor_target);


#endif