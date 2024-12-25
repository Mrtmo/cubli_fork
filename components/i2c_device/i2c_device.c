#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_device.h"

#define I2C_DEVICE_MAX_TRANS_UNIT (48)

static const char TAG[] = "i2c-i2c";

esp_err_t i2c_dev_init(i2c_master_bus_handle_t bus_handle, const i2c_dev_cfg_t *i2c_config, i2c_dev_handle_t *i2c_handle)
{
    esp_err_t ret = ESP_OK;
    i2c_dev_handle_t out_handle;
    out_handle = (i2c_dev_handle_t)calloc(1, sizeof(i2c_dev_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c eeprom device");

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_config->i2c_device.scl_speed_hz,
        .device_address = i2c_config->i2c_device.device_address,
    };

    if (out_handle->i2c_dev == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev), err, TAG, "i2c new bus failed");
    }

    out_handle->buffer = (uint8_t*)calloc(1, i2c_config->addr_wordlen + I2C_DEVICE_MAX_TRANS_UNIT);
    ESP_GOTO_ON_FALSE(out_handle->buffer, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c i2c device buffer");

    out_handle->addr_wordlen = i2c_config->addr_wordlen;
    out_handle->write_time_ms = i2c_config->write_time_ms;
    *i2c_handle = out_handle;

    return ESP_OK;

err:
    if (out_handle && out_handle->i2c_dev) {
        i2c_master_bus_rm_device(out_handle->i2c_dev);
    }
    free(out_handle);
    return ret;
}

esp_err_t i2c_dev_write(i2c_dev_handle_t i2c_handle, uint32_t address, const uint8_t *data, uint32_t size)
{
    ESP_RETURN_ON_FALSE(i2c_handle, ESP_ERR_NO_MEM, TAG, "no mem for buffer");
    for (int i = 0; i < i2c_handle->addr_wordlen; i++) {
        i2c_handle->buffer[i] = (address & (0xff << ((i2c_handle->addr_wordlen - 1 - i) * 8))) >> ((i2c_handle->addr_wordlen - 1 - i) * 8);
    }
    memcpy(i2c_handle->buffer + i2c_handle->addr_wordlen, data, size);

    return i2c_master_transmit(i2c_handle->i2c_dev, i2c_handle->buffer, i2c_handle->addr_wordlen + size, -1);
}

esp_err_t i2c_dev_read(i2c_dev_handle_t i2c_handle, uint32_t address, uint8_t *data, uint32_t size)
{
    ESP_RETURN_ON_FALSE(i2c_handle, ESP_ERR_NO_MEM, TAG, "no mem for buffer");
    for (int i = 0; i < i2c_handle->addr_wordlen; i++) {
        i2c_handle->buffer[i] = (address & (0xff << ((i2c_handle->addr_wordlen - 1 - i) * 8))) >> ((i2c_handle->addr_wordlen - 1 - i) * 8);
    }

    return i2c_master_transmit_receive(i2c_handle->i2c_dev, i2c_handle->buffer, i2c_handle->addr_wordlen, data, size, 500);
}

void i2c_dev_wait_idle(i2c_dev_handle_t i2c_handle)
{
    // This is time for i2c Self-Timed Write Cycle
    vTaskDelay(pdMS_TO_TICKS(i2c_handle->write_time_ms));
}
