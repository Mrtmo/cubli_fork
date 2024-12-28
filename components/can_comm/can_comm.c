#include "can_comm.h"
#include "driver/twai.h"

//Example Configurations
#define NO_OF_MSGS              100
#define NO_OF_ITERS             3
#define TX_GPIO_NUM             GPIO_NUM_15
#define RX_GPIO_NUM             GPIO_NUM_13

#define MSG_ID                  0x555   //11 bit standard format ID
#define EXAMPLE_TAG             "TWAI Self Test"



static void can_transmit(uint32_t id, uint8_t tx_data[], uint8_t len)
{
    ESP_ERROR_CHECK(twai_start());

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
        .data = tx_data,
    };
    ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));

    ESP_ERROR_CHECK(twai_stop());               //Stop the TWAI Driver
}

esp_err_t can_init()
{
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
    //Filter all other IDs except MSG_ID
    twai_filter_config_t f_config = {.acceptance_code = (MSG_ID << 21),
                                                .acceptance_mask = ~(TWAI_STD_ID_MASK << 21),
                                                .single_filter = true
                                                };
    //Set to NO_ACK mode due to self testing with single module
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
    return twai_driver_install(&g_config, &t_config, &f_config);
}
