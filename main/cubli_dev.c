
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "esp_timer.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "as5600.h"

/** as5600 配置 */
#define SCL_IO_PIN GPIO_NUM_14
#define SDA_IO_PIN GPIO_NUM_4
#define MASTER_FREQUENCY 100000
#define PORT_NUMBER -1
#define LENGTH 2

/** pwm引脚设置 */
#define _constrain(amt, low, high) (amt < low) ? low : ((amt > high) ? high : amt)
/** 一些参数 */
int pwm1 = GPIO_NUM_25;             // 电机驱动引脚1
int pwm2 = GPIO_NUM_26;             // 电机驱动引脚2
int pwm3 = GPIO_NUM_27;             // 电机驱动引脚3
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

void motor_pwm_init(void)
{
    gpio_config_t pwm_cfg = {
        .pin_bit_mask = 1 << pwm1 | 1 << pwm2 | 1 << pwm3,
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

    ledc_channel_config_t motor_ledc_channal_config_t = {
        .channel = LEDC_CHANNEL_0,
        .gpio_num = pwm1,
        .timer_sel = LEDC_TIMER_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&motor_ledc_channal_config_t);
    motor_ledc_channal_config_t.channel = LEDC_CHANNEL_1;
    motor_ledc_channal_config_t.gpio_num = pwm2;
    ledc_channel_config(&motor_ledc_channal_config_t);
    motor_ledc_channal_config_t.channel = LEDC_CHANNEL_2;
    motor_ledc_channal_config_t.gpio_num = pwm3;
    ledc_channel_config(&motor_ledc_channal_config_t);
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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dc_a);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dc_b);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, dc_c);
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

// 开环速度函数
float velocityOpenloop(float target_velocity)
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
    float Uq = voltage_power_supply / 3;

    set_phase_voltage(Uq, 0, _electrical_angle(shaft_angle, 7));

    open_loop_timestamp = now_us; // 用于计算下一个时间间隔

    return Uq;
}

static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0)
        {
            printf("\n");
        }
    }
    printf("%4d\n", (buf[0] << 8) + buf[1]);
}

void task_as5600_test(void *param)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_as5600_config_t as5600_config = {
        .as5600_device.scl_speed_hz = MASTER_FREQUENCY,
        .as5600_device.device_address = 0x36,
        .addr_wordlen = 1,
        .write_time_ms = 10,
    };

    i2c_as5600_handle_t as5600_handle;

    // uint8_t buf[LENGTH];
    // for (int i = 0; i < LENGTH; i++) {
    //     buf[i] = i;
    // }

    uint32_t block_addr = 0x0E;
    uint8_t read_buf[LENGTH];
    ESP_ERROR_CHECK(i2c_as5600_init(bus_handle, &as5600_config, &as5600_handle));
    ESP_LOGI("main", "as5600 init");

    while (1)
    {
        // ESP_ERROR_CHECK(i2c_as5600_write(as5600_handle, block_addr, buf, LENGTH));
        // // Needs wait for as5600 hardware done, referring from datasheet
        // i2c_as5600_wait_idle(as5600_handle);
        ESP_ERROR_CHECK(i2c_as5600_read(as5600_handle, block_addr, read_buf, LENGTH));
        disp_buf(read_buf, LENGTH);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    motor_pwm_init();
    // xTaskCreatePinnedToCore(task_as5600_test, "task_as5600", 2048 * 4, NULL, 3, NULL, 1);
    while(1)
    {
        velocityOpenloop(10);
    }
}
