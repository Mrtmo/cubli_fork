
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "mmpu6050.h"

#include "as5600.h"
#include "motor2204.h"
#include "button.h"
#include "can_comm.h"

/** as5600 配置 */
#define AS5600_SCL_IO_PIN GPIO_NUM_14
#define AS5600_SDA_IO_PIN GPIO_NUM_4
#define AS5600_ADDRESS 0x36
#define PORT_NUMBER -1

/** 按键引脚配置 */
#define MCU_SWITCH_1    GPIO_NUM_22
#define MCU_SWITCH_2    GPIO_NUM_18

/** motor 配置 */
gpio_num_t pwm1 = GPIO_NUM_25;       // 电机驱动引脚1
gpio_num_t pwm2 = GPIO_NUM_26;       // 电机驱动引脚2
gpio_num_t pwm3 = GPIO_NUM_27;       // 电机驱动引脚3
gpio_num_t pwm_enable = GPIO_NUM_33; // 驱动芯片使能引脚
// 
#define I2C_MASTER_SCL_IO 18      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 5      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM    I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define MPU6050_I2C_ADDRESS 0x68

void task_mpu_test(void * params)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
    };
    i2c_dev_cfg_t mpu_config = {
        .i2c_device.scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .i2c_device.device_address = MPU6050_I2C_ADDRESS,
        .addr_wordlen = 1,
        .write_time_ms = 100,
    };
    mpu6050_handle_t mpu_handle;
    mpu6050_create(&i2c_bus_config, &mpu_config, &mpu_handle);
    uint8_t data[8] = {0};
    mpu6050_get_deviceid(mpu_handle, data);
    printf("id = %d\n", data[0]);
}

i2c_as5600_handle_t as5600_handle;
void task_as5600_update(void *param)
{

    ESP_ERROR_CHECK(as5600_init(AS5600_ADDRESS, AS5600_SCL_IO_PIN, AS5600_SDA_IO_PIN, &as5600_handle));
    ESP_LOGI("main", "as5600 init");

    while (1)
    {
        ESP_ERROR_CHECK(as5600_update(as5600_handle));
        printf("%f, %f\n", as5600_get_angle(as5600_handle), as5600_get_angle_cycle(as5600_handle));
    }
}

void task_motor_test(void *params)
{
    ESP_ERROR_CHECK(as5600_init(AS5600_ADDRESS, AS5600_SCL_IO_PIN, AS5600_SDA_IO_PIN, &as5600_handle));
    motor_link_as5600(as5600_handle);
    motor_pwm_init(pwm1, pwm2, pwm3, pwm_enable);
    motor_enable();
    // while (1)
    // {
    //     velocity_openloop(10.f);
    // }
    weizhi_loop(0.f / 180 * M_PI);
}

void task_motor_velocity_openloop_test(void *params)
{
    ESP_ERROR_CHECK(as5600_init(AS5600_ADDRESS, AS5600_SCL_IO_PIN, AS5600_SDA_IO_PIN, &as5600_handle));
    motor_link_as5600(as5600_handle);
    motor_pwm_init(pwm1, pwm2, pwm3, pwm_enable);
    motor_enable();
    while (1)
    {
        velocity_openloop(10.f);
    }
    // weizhi_loop(0.f / 180 * M_PI);
}


void app_main(void)
{
    // task_mpu_test(NULL);
    xTaskCreatePinnedToCore(task_motor_test, "task_as5600", 2048 * 4, NULL, 3, NULL, 1);
    // xTaskCreatePinnedToCore(task_motor_test, "task_motor", 2048 * 4, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(task_btn_test, "task_btn", 2048 * 4, NULL, 3, NULL, 1);
    // xTaskCreatePinnedToCore(task_btn_test_queue, "task_btn", 2048 * 4, NULL, 3, NULL, 1);
}
