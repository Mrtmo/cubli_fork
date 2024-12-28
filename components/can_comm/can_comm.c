#include <string.h>

#include "can_comm.h"
#include "driver/twai.h"


//Example Configurations
#define NO_OF_MSGS              100
#define NO_OF_ITERS             3
#define TX_GPIO_NUM             GPIO_NUM_15
#define RX_GPIO_NUM             GPIO_NUM_13

#define MSG_ID                  0x555   //11 bit standard format ID
#define EXAMPLE_TAG             "TWAI Self Test"
#define TRANS_DELAY             portMAX_DELAY


static esp_err_t can_transmit(uint32_t id, uint8_t tx_data[], uint8_t len)
{
    twai_message_t tx_msg = {
        // Message type and format settings
        .extd = 0,              // Standard Format message (11-bit ID)
        .rtr = 0,               // Send a data frame
        .ss = 0,                // Not single shot
        .self = 0,              // Message is a self reception request (loopback)
        .dlc_non_comp = 0,      // DLC is less than 8
        // Message ID and payload
        .identifier = id,
        .data_length_code = len,
        .data = {0},
    };
    memcpy(tx_msg.data, tx_data, len);
    return twai_transmit(&tx_msg, TRANS_DELAY);
}

esp_err_t can_init()
{
    //Set to 
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
    //Accept all IDs
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    return twai_driver_install(&g_config, &t_config, &f_config);
}

esp_err_t can_enable()
{
    return twai_start();
}

esp_err_t can_disable()
{
    return twai_stop();               //Stop the TWAI Driver
}
