#ifndef _AS5600_H_
#define _AS5600_H_

#include <stdint.h>

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_device_config_t as5600_device;  /*!< Configuration for as5600 device */
    uint8_t addr_wordlen;               /*!< block address wordlen */
    uint8_t write_time_ms;              /*!< as5600 write time, typically 10ms*/
} i2c_as5600_config_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;      /*!< I2C device handle */
    uint8_t addr_wordlen;                 /*!< block address wordlen */
    uint8_t *buffer;                      /*!< I2C transaction buffer */
    uint8_t write_time_ms;                /*!< I2C as5600 write time(ms)*/
}i2c_as5600_t;


/* handle of EEPROM device */
typedef i2c_as5600_t *i2c_as5600_handle_t;

/**
 * @brief Init an as5600 device.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] as5600_config Configuration of EEPROM
 * @param[out] as5600_handle Handle of EEPROM
 * @return ESP_OK: Init success. ESP_FAIL: Not success.
 */
esp_err_t i2c_as5600_init(i2c_master_bus_handle_t bus_handle, const i2c_as5600_config_t *as5600_config, i2c_as5600_handle_t *as5600_handle);

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

/**
 * @brief Wait as5600 finish. Typically 5ms
 *
 * @param eeprom_handle as5600 handle
 */
void i2c_as5600_wait_idle(i2c_as5600_handle_t as5600_handle);

#ifdef __cplusplus
}
#endif

#endif
