#ifndef _CAN_COMM_H_
#define _CAN_COMM_H_

#include <stdint.h>
#include "esp_check.h"

esp_err_t can_init();
esp_err_t can_enable();
esp_err_t can_disable();



#endif