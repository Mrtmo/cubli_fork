
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "as5600.h"
#include "motor2204.h"

/** as5600 配置 */
#define AS5600_SCL_IO_PIN GPIO_NUM_14
#define AS5600_SDA_IO_PIN GPIO_NUM_4
#define AS5600_ADDRESS 0x36
#define PORT_NUMBER -1
#define LENGTH 2

/** motor 配置 */
int pwm1 = GPIO_NUM_25;       // 电机驱动引脚1
int pwm2 = GPIO_NUM_26;       // 电机驱动引脚2
int pwm3 = GPIO_NUM_27;       // 电机驱动引脚3
int pwm_enable = GPIO_NUM_33; // 驱动芯片使能引脚

void task_as5600_test(void *param)
{
    i2c_as5600_handle_t as5600_handle;
    uint32_t block_addr = 0x0E;
    uint8_t read_buf[LENGTH];

    ESP_ERROR_CHECK(i2c_as5600_init(AS5600_ADDRESS, AS5600_SCL_IO_PIN, AS5600_SDA_IO_PIN, &as5600_handle));
    ESP_LOGI("main", "as5600 init");

    while (1)
    {
        ESP_ERROR_CHECK(i2c_as5600_read(as5600_handle, block_addr, read_buf, LENGTH));
        printf("%4d\n", (read_buf[0] << 8) + read_buf[1]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_motor_test(void *params)
{
    motor_pwm_init(pwm1, pwm2, pwm3, pwm_enable);
    while (1)
    {
        velocity_openloop(10);
    }
}

void app_main(void)
{
    xTaskCreatePinnedToCore(task_as5600_test, "task_as5600", 2048 * 4, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(task_motor_test, "task_motor", 2048 * 4, NULL, 3, NULL, 1);
}
