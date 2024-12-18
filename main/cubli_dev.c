
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "as5600.h"

#define SCL_IO_PIN          GPIO_NUM_14
#define SDA_IO_PIN          GPIO_NUM_4
#define MASTER_FREQUENCY    100000
#define PORT_NUMBER         -1
#define LENGTH              2

static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("%4d\n", (buf[0] << 8) + buf[1]);
}

void task_as5600_test(void* param)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_as5600_config_t as5600_config = {
        .as5600_device.scl_speed_hz = MASTER_FREQUENCY,
        .as5600_device.device_address = 0x36,
        .addr_wordlen = 1,
        .write_time_ms = 10,
    };

    i2c_as5600_handle_t as5600_handle;

    // uint8_t buf[LENGTH];
    // for (int i = 0; i < LENGTH; i++) {
    //     buf[i] = i;
    // }

    uint32_t block_addr = 0x0E;
    uint8_t read_buf[LENGTH];
    ESP_ERROR_CHECK(i2c_as5600_init(bus_handle, &as5600_config, &as5600_handle));
    ESP_LOGI("main", "as5600 init");

    while (1) {
        // ESP_ERROR_CHECK(i2c_as5600_write(as5600_handle, block_addr, buf, LENGTH));
        // // Needs wait for as5600 hardware done, referring from datasheet
        // i2c_as5600_wait_idle(as5600_handle);
        ESP_ERROR_CHECK(i2c_as5600_read(as5600_handle, block_addr, read_buf, LENGTH));
        disp_buf(read_buf, LENGTH);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{

    xTaskCreatePinnedToCore(task_as5600_test, "task_as5600", 2048*4, NULL, 3, NULL, 1);
}
