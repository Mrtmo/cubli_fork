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

/* handle of as5600 device */
typedef i2c_dev_t *i2c_as5600_handle_t;

/**
 * @brief Init an as5600 device.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] as5600_config Configuration of EEPROM
 * @param[out] as5600_handle Handle of EEPROM
 * @return ESP_OK: Init success. ESP_FAIL: Not success.
 */
extern esp_err_t as5600_init(uint8_t address, uint8_t scl_pin, uint8_t sda_pin, i2c_as5600_handle_t *as5600_handle);

extern esp_err_t as5600_update(i2c_as5600_handle_t as5600_handle);

extern float as5600_get_angle(i2c_as5600_handle_t as5600_handle);

extern float as5600_get_angle_cycle(i2c_as5600_handle_t as5600_handle);


#ifdef __cplusplus
}
#endif

#endif
