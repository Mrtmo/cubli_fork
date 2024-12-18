#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "as5600.h"

#define I2C_AS5600_MAX_TRANS_UNIT (48)

static const char TAG[] = "i2c-as5600";

esp_err_t i2c_as5600_init(i2c_master_bus_handle_t bus_handle, const i2c_as5600_config_t *as5600_config, i2c_as5600_handle_t *as5600_handle)
{
    esp_err_t ret = ESP_OK;
    i2c_as5600_handle_t out_handle;
    out_handle = (i2c_as5600_handle_t)calloc(1, sizeof(i2c_as5600_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c eeprom device");

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = as5600_config->as5600_device.scl_speed_hz,
        .device_address = as5600_config->as5600_device.device_address,
    };

    if (out_handle->i2c_dev == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev), err, TAG, "i2c new bus failed");
    }

    out_handle->buffer = (uint8_t*)calloc(1, as5600_config->addr_wordlen + I2C_AS5600_MAX_TRANS_UNIT);
    ESP_GOTO_ON_FALSE(out_handle->buffer, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c as5600 device buffer");

    out_handle->addr_wordlen = as5600_config->addr_wordlen;
    out_handle->write_time_ms = as5600_config->write_time_ms;
    *as5600_handle = out_handle;

    return ESP_OK;

err:
    if (out_handle && out_handle->i2c_dev) {
        i2c_master_bus_rm_device(out_handle->i2c_dev);
    }
    free(out_handle);
    return ret;
}

esp_err_t i2c_as5600_write(i2c_as5600_handle_t as5600_handle, uint32_t address, const uint8_t *data, uint32_t size)
{
    ESP_RETURN_ON_FALSE(as5600_handle, ESP_ERR_NO_MEM, TAG, "no mem for buffer");
    for (int i = 0; i < as5600_handle->addr_wordlen; i++) {
        as5600_handle->buffer[i] = (address & (0xff << ((as5600_handle->addr_wordlen - 1 - i) * 8))) >> ((as5600_handle->addr_wordlen - 1 - i) * 8);
    }
    memcpy(as5600_handle->buffer + as5600_handle->addr_wordlen, data, size);

    return i2c_master_transmit(as5600_handle->i2c_dev, as5600_handle->buffer, as5600_handle->addr_wordlen + size, -1);
}

esp_err_t i2c_as5600_read(i2c_as5600_handle_t as5600_handle, uint32_t address, uint8_t *data, uint32_t size)
{
    ESP_RETURN_ON_FALSE(as5600_handle, ESP_ERR_NO_MEM, TAG, "no mem for buffer");
    for (int i = 0; i < as5600_handle->addr_wordlen; i++) {
        as5600_handle->buffer[i] = (address & (0xff << ((as5600_handle->addr_wordlen - 1 - i) * 8))) >> ((as5600_handle->addr_wordlen - 1 - i) * 8);
    }

    return i2c_master_transmit_receive(as5600_handle->i2c_dev, as5600_handle->buffer, as5600_handle->addr_wordlen, data, size, 500);
}

void i2c_as5600_wait_idle(i2c_as5600_handle_t as5600_handle)
{
    // This is time for as5600 Self-Timed Write Cycle
    vTaskDelay(pdMS_TO_TICKS(as5600_handle->write_time_ms));
}
