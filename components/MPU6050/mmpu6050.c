#include "mmpu6050.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/* MPU6050 register */
#define MPU6050_GYRO_CONFIG         0x1Bu
#define MPU6050_ACCEL_CONFIG        0x1Cu
#define MPU6050_INTR_PIN_CFG         0x37u
#define MPU6050_INTR_ENABLE          0x38u
#define MPU6050_INTR_STATUS          0x3Au
#define MPU6050_ACCEL_XOUT_H        0x3Bu
#define MPU6050_GYRO_XOUT_H         0x43u
#define MPU6050_TEMP_XOUT_H         0x41u
#define MPU6050_PWR_MGMT_1          0x6Bu
#define MPU6050_WHO_AM_I            0x75u

typedef struct {
    i2c_port_t bus;
    gpio_num_t int_pin;

    uint16_t dev_addr;
    uint32_t counter;
    float dt;  /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
} mpu6050_dev_t;

esp_err_t mpu6050_create(i2c_master_bus_config_t* i2c_bus_config, i2c_dev_cfg_t* mpu_config, mpu6050_handle_t * mpu6050_handle)
{

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(i2c_bus_config, &bus_handle));

    return i2c_dev_init(bus_handle, mpu_config, mpu6050_handle);
}

static esp_err_t mpu6050_write(mpu6050_handle_t mpu6050_handle, uint32_t address, const uint8_t *data, uint32_t size)
{
    return i2c_dev_write(mpu6050_handle, address, data, size);
}

static esp_err_t mpu6050_read(mpu6050_handle_t mpu6050_handle, uint32_t address, const uint8_t *data, uint32_t size)
{
    return i2c_dev_read(mpu6050_handle, address, data, size);
}

esp_err_t mpu6050_get_deviceid(mpu6050_handle_t mpu6050_handle, uint8_t *const deviceid)
{
    return mpu6050_read(mpu6050_handle, MPU6050_WHO_AM_I, deviceid, 1);
}

esp_err_t mpu6050_wake_up(mpu6050_handle_t mpu6050_handle)
{
    esp_err_t ret;
    uint8_t tmp = 0;
    ret = mpu6050_read(mpu6050_handle, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp &= (~BIT6);
    ret = mpu6050_write(mpu6050_handle, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_sleep(mpu6050_handle_t mpu6050_handle)
{
    esp_err_t ret;
    uint8_t tmp = 0;
    ret = mpu6050_read(mpu6050_handle, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT6;
    ret = mpu6050_write(mpu6050_handle, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_config(mpu6050_handle_t mpu6050_handle, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t config_regs[2] = {gyro_fs << 3,  acce_fs << 3};
    return mpu6050_write(mpu6050_handle, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
}

esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t mpu6050_handle, float *const acce_sensitivity)
{
    esp_err_t ret;
    uint8_t acce_fs = 0;
    ret = mpu6050_read(mpu6050_handle, MPU6050_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
    case ACCE_FS_2G:
        *acce_sensitivity = 16384;
        break;

    case ACCE_FS_4G:
        *acce_sensitivity = 8192;
        break;

    case ACCE_FS_8G:
        *acce_sensitivity = 4096;
        break;

    case ACCE_FS_16G:
        *acce_sensitivity = 2048;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t mpu6050_handle, float *const gyro_sensitivity)
{
    esp_err_t ret;
    uint8_t gyro_fs = 0;
    ret = mpu6050_read(mpu6050_handle, MPU6050_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
    case GYRO_FS_250DPS:
        *gyro_sensitivity = 131;
        break;

    case GYRO_FS_500DPS:
        *gyro_sensitivity = 65.5;
        break;

    case GYRO_FS_1000DPS:
        *gyro_sensitivity = 32.8;
        break;

    case GYRO_FS_2000DPS:
        *gyro_sensitivity = 16.4;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t mpu6050_handle, mpu6050_raw_acce_value_t *const raw_acce_value)
{
    uint8_t data_rd[6] = {0};
    esp_err_t ret = mpu6050_read(mpu6050_handle, MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t mpu6050_handle, mpu6050_raw_gyro_value_t *const raw_gyro_value)
{
    uint8_t data_rd[6] = {0};
    esp_err_t ret = mpu6050_read(mpu6050_handle, MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

    return ret;
}

esp_err_t mpu6050_get_acce(mpu6050_handle_t mpu6050_handle, mpu6050_acce_value_t *const acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    mpu6050_raw_acce_value_t raw_acce;

    ret = mpu6050_get_acce_sensitivity(mpu6050_handle, &acce_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6050_get_raw_acce(mpu6050_handle, &raw_acce);
    if (ret != ESP_OK) {
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_gyro(mpu6050_handle_t mpu6050_handle, mpu6050_gyro_value_t *const gyro_value)
{
    esp_err_t ret;
    float gyro_sensitivity;
    mpu6050_raw_gyro_value_t raw_gyro;

    ret = mpu6050_get_gyro_sensitivity(mpu6050_handle, &gyro_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6050_get_raw_gyro(mpu6050_handle, &raw_gyro);
    if (ret != ESP_OK) {
        return ret;
    }

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_temp(mpu6050_handle_t mpu6050_handle, mpu6050_temp_value_t *const temp_value)
{
    uint8_t data_rd[2] = {0};
    esp_err_t ret = mpu6050_read(mpu6050_handle, MPU6050_TEMP_XOUT_H, data_rd, sizeof(data_rd));
    temp_value->temp = (int16_t)((data_rd[0] << 8) | (data_rd[1])) / 340.00 + 36.53;
    return ret;
}

// esp_err_t mpu6050_complimentory_filter(mpu6050_handle_t sensor, const mpu6050_acce_value_t *const acce_value,
//                                        const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
// {
//     float acce_angle[2];
//     float gyro_angle[2];
//     float gyro_rate[2];
//     mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;

//     sens->counter++;
//     if (sens->counter == 1) {
//         acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
//         acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
//         complimentary_angle->roll = acce_angle[0];
//         complimentary_angle->pitch = acce_angle[1];
//         gettimeofday(sens->timer, NULL);
//         return ESP_OK;
//     }

//     struct timeval now, dt_t;
//     gettimeofday(&now, NULL);
//     timersub(&now, sens->timer, &dt_t);
//     sens->dt = (float) (dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
//     gettimeofday(sens->timer, NULL);

//     acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
//     acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);

//     gyro_rate[0] = gyro_value->gyro_x;
//     gyro_rate[1] = gyro_value->gyro_y;
//     gyro_angle[0] = gyro_rate[0] * sens->dt;
//     gyro_angle[1] = gyro_rate[1] * sens->dt;

//     complimentary_angle->roll = (ALPHA * (complimentary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
//     complimentary_angle->pitch = (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);

//     return ESP_OK;
// }