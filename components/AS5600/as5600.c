
#include "as5600.h"

#define MASTER_FREQUENCY 100000

// #define I2C_AS5600_MAX_TRANS_UNIT (48)

// static const char TAG[] = "i2c-as5600";

esp_err_t i2c_as5600_init(uint8_t address, uint8_t scl_pin, uint8_t sda_pin, i2c_as5600_handle_t *as5600_handle)
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

esp_err_t i2c_as5600_write(i2c_as5600_handle_t as5600_handle, uint32_t address, const uint8_t *data, uint32_t size)
{
    return i2c_dev_write(as5600_handle, address, data, size);
}

esp_err_t i2c_as5600_read(i2c_as5600_handle_t as5600_handle, uint32_t address, uint8_t *data, uint32_t size)
{
    return i2c_dev_read(as5600_handle, address, data, size);
}

// void i2c_as5600_wait_idle(i2c_as5600_handle_t as5600_handle)
// {
//     // This is time for as5600 Self-Timed Write Cycle
//     vTaskDelay(pdMS_TO_TICKS(as5600_handle->write_time_ms));
// }
