#include <math.h>

#include "as5600.h"
#include "esp_check.h"

#define MASTER_FREQUENCY 100000

// #define I2C_AS5600_MAX_TRANS_UNIT (48)

// static const char TAG[] = "i2c-as5600";
#define AS5600_ADDRESS_ANGLE 0x0E
#define LENGTH 2

float angle = 0.0f;
float angle_cycle = 0.0f;

esp_err_t as5600_init(uint8_t address, uint8_t scl_pin, uint8_t sda_pin, i2c_as5600_handle_t *as5600_handle)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_as5600_config_t as5600_config = {
        .i2c_device.scl_speed_hz = MASTER_FREQUENCY,
        .i2c_device.device_address = address,
        .addr_wordlen = 1,
        .write_time_ms = 10,
    };

    return i2c_dev_init(bus_handle, &as5600_config, as5600_handle);
}

static esp_err_t i2c_as5600_write(i2c_as5600_handle_t as5600_handle, uint32_t address, const uint8_t *data, uint32_t size)
{
    return i2c_dev_write(as5600_handle, address, data, size);
}

static esp_err_t i2c_as5600_read(i2c_as5600_handle_t as5600_handle, uint32_t address, uint8_t *data, uint32_t size)
{
    return i2c_dev_read(as5600_handle, address, data, size);
}

// void i2c_as5600_wait_idle(i2c_as5600_handle_t as5600_handle)
// {
//     // This is time for as5600 Self-Timed Write Cycle
//     vTaskDelay(pdMS_TO_TICKS(as5600_handle->write_time_ms));
// }

esp_err_t as5600_update(i2c_as5600_handle_t as5600_handle)
{
    uint8_t read_buf[LENGTH];
    esp_err_t ret;
    float temp_angle;
    float minus;
    const float thr = M_PI / 2.0f;

    ret = i2c_as5600_read(as5600_handle, AS5600_ADDRESS_ANGLE, read_buf, LENGTH);
    if (ret != ESP_OK)
    {
        return ret;
    }
    temp_angle = ((read_buf[0] << 8) + read_buf[1]) * 2 * M_PI / 4096.0f;

    minus = temp_angle - angle;
    angle_cycle += minus;
    if (minus > thr)
    {
        angle_cycle -= M_PI * 2;
    }
    else if (minus < -thr)
    {
        angle_cycle += M_PI * 2;
    }
    angle = temp_angle;

    return ret;
}

float as5600_get_angle(i2c_as5600_handle_t as5600_handle)
{
    return angle;
}

float as5600_get_angle_cycle(i2c_as5600_handle_t as5600_handle)
{
    return angle_cycle;
}