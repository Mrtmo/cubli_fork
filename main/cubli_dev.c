
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "mpu6050.h"

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
// 
#define I2C_MASTER_SCL_IO 18      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 5      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM    I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050 test";
static mpu6050_handle_t mpu6050 = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    // i2c_config_t conf;
    // conf.mode = I2C_MODE_MASTER;
    // conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    // conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    // conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    // conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    // conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    // conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    // esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    // TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    // ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    // TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_dev_cfg_t mpu_config = {
        .i2c_device.scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .i2c_device.device_address = MPU6050_I2C_ADDRESS,
        .addr_wordlen = 1,
        .write_time_ms = 10,
    };
    i2c_dev_handle_t mpu_handle;

    i2c_dev_init(bus_handle, &mpu_config, &mpu_handle);
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void task_mpu_test(void * params)
{
    esp_err_t ret;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    i2c_sensor_mpu6050_init();

    // ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    // TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");

    // ret = mpu6050_get_acce(mpu6050, &acce);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    // ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

    // ret = mpu6050_get_gyro(mpu6050, &gyro);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    // ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

    // ret = mpu6050_get_temp(mpu6050, &temp);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    // ESP_LOGI(TAG, "t:%.2f \n", temp.temp);

    // mpu6050_delete(mpu6050);
    // ret = i2c_driver_delete(I2C_MASTER_NUM);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
}

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
    task_mpu_test(NULL);
    // xTaskCreatePinnedToCore(task_as5600_test, "task_as5600", 2048 * 4, NULL, 3, NULL, 1);
    // xTaskCreatePinnedToCore(task_motor_test, "task_motor", 2048 * 4, NULL, 3, NULL, 1);
}
