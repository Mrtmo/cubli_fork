#ifndef _MMPU6050_H_
#define _MMPU6050_H_

#include <stdint.h>
#include "esp_check.h"
#include "i2c_device.h"

// 加速度计量程
typedef enum {
    ACCE_FS_2G  = 0,     /*!< Accelerometer full scale range is +/- 2g */
    ACCE_FS_4G  = 1,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_8G  = 2,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_16G = 3,     /*!< Accelerometer full scale range is +/- 16g */
} mpu6050_acce_fs_t;

// 陀螺仪量程
typedef enum {
    GYRO_FS_250DPS  = 0,     /*!< Gyroscope full scale range is +/- 250 degree per sencond */
    GYRO_FS_500DPS  = 1,     /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    GYRO_FS_1000DPS = 2,     /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    GYRO_FS_2000DPS = 3,     /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
} mpu6050_gyro_fs_t;

// 
typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} mpu6050_raw_acce_value_t;

typedef struct {
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} mpu6050_raw_gyro_value_t;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
} mpu6050_acce_value_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_gyro_value_t;

typedef struct {
    float temp;
} mpu6050_temp_value_t;

typedef i2c_dev_t * mpu6050_handle_t;

extern esp_err_t mpu6050_create(i2c_master_bus_config_t* i2c_bus_config, i2c_dev_cfg_t* mpu_config, mpu6050_handle_t* mpu_handle);

// 获取 mpu6050 i2c地址
extern esp_err_t mpu6050_get_deviceid(mpu6050_handle_t mpu_handle, uint8_t *const deviceid);

// 设置 mpu6050 进入唤醒模式
extern esp_err_t mpu6050_wake_up(mpu6050_handle_t mpu_handle);

// 设置 mpu6050 进入睡眠模式
extern esp_err_t mpu6050_sleep(mpu6050_handle_t mpu_handle);

// mpu6050 陀螺仪与加速度计量程设置
extern esp_err_t mpu6050_config(mpu6050_handle_t mpu_handle, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs);

// 获取加速度计的精度
extern esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t mpu_handle, float *const acce_sensitivity);

// 获取陀螺仪的精度
extern esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t mpu_handle, float *const gyro_sensitivity);

// 获取加速度计的原始值
extern esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t mpu6050_handle, mpu6050_raw_acce_value_t *const raw_acce_value);

// 获取陀螺仪的原始值
extern esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t mpu6050_handle, mpu6050_raw_gyro_value_t *const raw_gyro_value);

// 获取加速度计的实际值
extern esp_err_t mpu6050_get_acce(mpu6050_handle_t mpu6050_handle, mpu6050_acce_value_t *const acce_value);

// 获取陀螺仪的实际值
extern esp_err_t mpu6050_get_gyro(mpu6050_handle_t mpu6050_handle, mpu6050_gyro_value_t *const gyro_value);

// 获取温度值
extern esp_err_t mpu6050_get_temp(mpu6050_handle_t mpu6050_handle, mpu6050_temp_value_t *const temp_value);

#endif