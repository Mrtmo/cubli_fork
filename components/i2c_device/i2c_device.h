#ifndef _I2C_DEVICE_H_
#define _I2C_DEVICE_H_

#include <stdint.h>

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_device_config_t i2c_device;  /*!< Configuration for i2c device */
    uint8_t addr_wordlen;               /*!< block address wordlen */
    uint8_t write_time_ms;              /*!< i2c write time, typically 10ms*/
} i2c_dev_cfg_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;      /*!< I2C device handle */
    uint8_t addr_wordlen;                 /*!< block address wordlen */
    uint8_t *buffer;                      /*!< I2C transaction buffer */
    uint8_t write_time_ms;                /*!< I2C i2c write time(ms)*/
}i2c_dev_t;


/* handle of EEPROM device */
typedef i2c_dev_t *i2c_dev_handle_t;

/**
 * @brief Init an i2c device.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] i2c_config Configuration of EEPROM
 * @param[out] i2c_handle Handle of EEPROM
 * @return ESP_OK: Init success. ESP_FAIL: Not success.
 */
esp_err_t i2c_dev_init(i2c_master_bus_handle_t bus_handle, const i2c_dev_cfg_t *i2c_config, i2c_dev_handle_t *i2c_handle);

/**
 * @brief Write data to i2c
 *
 * @param[in] i2c_handle i2c handle
 * @param[in] address Block address inside i2c
 * @param[in] data Data to write
 * @param[in] size Data write size
 * @return ESP_OK: Write success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_dev_write(i2c_dev_handle_t i2c_handle, uint32_t address, const uint8_t *data, uint32_t size);

/**
 * @brief Read data from i2c
 *
 * @param i2c_handle i2c handle
 * @param address Block address inside i2c
 * @param data Data read from i2c
 * @param size Data read size
 * @return ESP_OK: Read success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_dev_read(i2c_dev_handle_t i2c_handle, uint32_t address, uint8_t *data, uint32_t size);

/**
 * @brief Wait i2c finish. Typically 5ms
 *
 * @param eeprom_handle i2c handle
 */
void i2c_dev_wait_idle(i2c_dev_handle_t i2c_handle);

#ifdef __cplusplus
}
#endif

#endif
