#ifndef _AS5600_H_
#define _AS5600_H_

#include <stdint.h>

#include <stdint.h>
#include "esp_err.h"
#include "i2c_device.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef i2c_dev_cfg_t i2c_as5600_config_t;

/* handle of EEPROM device */
typedef i2c_dev_t *i2c_as5600_handle_t;

/**
 * @brief Init an as5600 device.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] as5600_config Configuration of EEPROM
 * @param[out] as5600_handle Handle of EEPROM
 * @return ESP_OK: Init success. ESP_FAIL: Not success.
 */
esp_err_t i2c_as5600_init(uint8_t address, uint8_t scl_pin, uint8_t sda_pin, i2c_as5600_handle_t *as5600_handle);

/**
 * @brief Write data to as5600
 *
 * @param[in] as5600_handle as5600 handle
 * @param[in] address Block address inside as5600
 * @param[in] data Data to write
 * @param[in] size Data write size
 * @return ESP_OK: Write success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_as5600_write(i2c_as5600_handle_t as5600_handle, uint32_t address, const uint8_t *data, uint32_t size);

/**
 * @brief Read data from as5600
 *
 * @param as5600_handle as5600 handle
 * @param address Block address inside as5600
 * @param data Data read from as5600
 * @param size Data read size
 * @return ESP_OK: Read success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_as5600_read(i2c_as5600_handle_t as5600_handle, uint32_t address, uint8_t *data, uint32_t size);

// /**
//  * @brief Wait as5600 finish. Typically 5ms
//  *
//  * @param eeprom_handle as5600 handle
//  */
// void i2c_as5600_wait_idle(i2c_as5600_handle_t as5600_handle);

#ifdef __cplusplus
}
#endif

#endif
