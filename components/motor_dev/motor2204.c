#include "math.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "motor2204.h"

/** pwm引脚设置 */
#define _constrain(amt, low, high) (amt < low) ? low : ((amt > high) ? high : amt)

float voltage_power_supply = 12.0;  // 电机额定电压
float voltage_power_limit = 12.0;   // 电机额定电压
int pole_pairs = 7;                 // 电机极对数
float Ua = 0, Ub = 0, Uc = 0;       // 三相电电压
float dc_a = 0, dc_b = 0, dc_c = 0; // 三相电占空比
float U_alpha = 0, U_beta = 0;      // 直角坐标系电压分量
float Ud = 0, Uq = 0;               // 帕克变换前磁极直角坐标系分量
float zero_electric_angle = 0;      // 0电度角偏移量
float open_loop_timestamp = 0;      // 开环控制时间戳
float shaft_angle = 0;              // 机械角度
gpio_num_t g_pwm_enable = -1;

void motor_pwm_init(gpio_num_t pwm1, gpio_num_t pwm2, gpio_num_t pwm3, gpio_num_t pwm_enable)
{
    g_pwm_enable = pwm_enable;
    gpio_config_t pwm_cfg = {
        .pin_bit_mask = 1 << pwm1 | 1 << pwm2 | 1 << pwm3 | 1llu << pwm_enable,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwm_cfg);

    ledc_timer_config_t motor_ledc_config_t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
        .freq_hz = 5000,
        .duty_resolution = LEDC_TIMER_12_BIT,
    };
    ledc_timer_config(&motor_ledc_config_t);
    motor_ledc_config_t.timer_num = LEDC_TIMER_1;
    ledc_timer_config(&motor_ledc_config_t);
    motor_ledc_config_t.timer_num = LEDC_TIMER_2;
    ledc_timer_config(&motor_ledc_config_t);

    ledc_channel_config_t motor_ledc_channal_config_t = {
        .channel = LEDC_CHANNEL_0,
        .gpio_num = pwm1,
        .timer_sel = LEDC_TIMER_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&motor_ledc_channal_config_t);
    motor_ledc_channal_config_t.gpio_num = pwm2;
    motor_ledc_channal_config_t.channel = LEDC_CHANNEL_1;
    motor_ledc_channal_config_t.timer_sel = LEDC_TIMER_1;
    ledc_channel_config(&motor_ledc_channal_config_t);
    motor_ledc_channal_config_t.gpio_num = pwm3;
    motor_ledc_channal_config_t.channel = LEDC_CHANNEL_2;
    motor_ledc_channal_config_t.timer_sel = LEDC_TIMER_2;
    ledc_channel_config(&motor_ledc_channal_config_t);
    motor_disable();
}

// 电角度求解
float _electrical_angle(float shaft_angle, int pole_pairs)
{
    return shaft_angle * pole_pairs;
}

// 电角度归一化
float _normalize_angle(float angle)
{
    float n_angle = fmod(angle, 2 * M_PI);
    return (n_angle > 0) ? n_angle : n_angle + 2 * M_PI;
}

// 设置 pwm 到控制器输出
void set_pwm(float Ua, float Ub, float Uc)
{
    Ua = _constrain(Ua, 0.0f, voltage_power_limit);
    Ub = _constrain(Ub, 0.0f, voltage_power_limit);
    Uc = _constrain(Uc, 0.0f, voltage_power_limit);
    // 计算占空比
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1);
    // 设置pwm输出
    // printf("duty: a-b-c: %f, %f, %f\n", dc_a, dc_b, dc_c);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dc_a * 4095);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dc_b * 4095);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, dc_c * 4095);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

// 设置三相电压
void set_phase_voltage(float Ud, float Uq, float angle)
{
    // 计算归一化电度角
    angle = _normalize_angle(angle + zero_electric_angle);
    // 帕克逆变换
    U_alpha = Ud * cos(angle) - Uq * sin(angle);
    U_beta = Ud * sin(angle) + Uq * cos(angle);
    // 克拉克逆变换
    Ua = U_alpha + voltage_power_supply / 2;
    Ub = (sqrt(3) * U_beta - U_alpha) / 2 + voltage_power_supply / 2;
    Uc = (-U_alpha - sqrt(3) * U_beta) / 2 + voltage_power_supply / 2;
    // 设置 pwm 输出
    set_pwm(Ua, Ub, Uc);
}

void motor_enable(void)
{
    gpio_set_direction(g_pwm_enable, GPIO_MODE_OUTPUT);
    gpio_set_level(g_pwm_enable, 1);
    printf("en_pin = %d, %llx\n", g_pwm_enable, ((uint64_t)1) << g_pwm_enable);
    gpio_dump_io_configuration(stdout, ((uint64_t)1) << g_pwm_enable);
}

void motor_disable(void)
{
    gpio_set_direction(g_pwm_enable, GPIO_MODE_INPUT);
}


// 开环速度函数
float velocity_openloop(float target_velocity)
{
    unsigned long now_us = esp_timer_get_time(); // 获取从开启芯片以来的微秒数，它的精度是 4 微秒。 micros() 返回的是一个无符号长整型（unsigned long）的值
    // ESP_LOGI("main", "%ld", now_us);q1
    // 计算当前每个Loop的运行时间间隔
    float Ts = (now_us - open_loop_timestamp) * 1e-6f;

    // 由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常，因此需要进行修正。如果时间间隔小于等于零或大于 0.5 秒，则将其设置为一个较小的默认值，即 1e-3f
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
    shaft_angle = _normalize_angle(shaft_angle + target_velocity * Ts);
    // 以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
    // 如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

    // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
    // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
    Uq = voltage_power_supply / 3;

    set_phase_voltage(0, Uq, _electrical_angle(shaft_angle, 7));

    open_loop_timestamp = now_us; // 用于计算下一个时间间隔

    return Uq;
}
