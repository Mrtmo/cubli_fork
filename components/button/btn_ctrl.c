#include "button.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

// 按键事件组
static EventGroupHandle_t s_pressEvent;
static QueueHandle_t queue_handle;

#define SW_1_SHORT_EV BIT0 // sw1短按
#define SW_1_LONG_EV  BIT1  // sw1长按
#define SW_2_SHORT_EV BIT2 // sw2短按
#define SW_2_LONG_EV  BIT3  // sw2长按
#define SW_EV_BITS    (SW_1_SHORT_EV | SW_1_LONG_EV | SW_2_SHORT_EV | SW_2_LONG_EV)

#define BTN_GPIO_1 GPIO_NUM_22
#define BTN_GPIO_2 GPIO_NUM_19

typedef struct{
    uint8_t io_num;
    uint8_t type;
} queue_data_t;

/** 短按按键回调函数
 * @param 无
 * @return 无
 */
void IRAM_ATTR short_press_handle(void *arg)
{
    if((uint32_t)arg == BTN_GPIO_1)
    {
        xEventGroupSetBits(s_pressEvent, SW_1_SHORT_EV);
    }
    else
    {
        xEventGroupSetBits(s_pressEvent, SW_2_SHORT_EV);
    }
    // queue_data_t data = {
    //     .io_num = (uint8_t) arg,
    //     .type = 0,
    // };
    // xQueueSendFromISR(queue_handle, &data, NULL);
}

/** 长按按键回调函数
 * @param 无
 * @return 无
 */
void IRAM_ATTR long_press_handle(void *arg)
{
    if((uint32_t)arg == BTN_GPIO_1)
    {
        xEventGroupSetBits(s_pressEvent, SW_1_LONG_EV);
    }
    else
    {
        xEventGroupSetBits(s_pressEvent, SW_2_LONG_EV);
    }
    // queue_data_t data = {
    //     .io_num = (uint8_t) arg,
    //     .type = 1,
    // };
    // xQueueSendFromISR(queue_handle, &data, NULL);
}

/*
 * @param 无
 * @return 无
 */
void task_btn_test(void *arg)
{
    s_pressEvent = xEventGroupCreate();
    EventBits_t ev;
    button_config_t btn_cfg =
        {
            .gpio_num = BTN_GPIO_1,         // gpio号
            .active_level = 0,              // 按下的电平
            .long_press_time = 3000,        // 长按时间
            .short_press_time = 1000,       // 短按时间
            .short_cb = short_press_handle, // 短按回调函数
            .long_cb = long_press_handle,   // 长按回调函数
            .arg = (void *)BTN_GPIO_1,
        };
    button_event_set(&btn_cfg); // 添加按键响应事件处理
    btn_cfg.gpio_num = BTN_GPIO_2;
    btn_cfg.arg = (void *)BTN_GPIO_2;
    button_event_set(&btn_cfg); // 添加按键响应事件处理
    while (1)
    {
        // 等待按键按下事件
        ev = xEventGroupWaitBits(s_pressEvent, SW_EV_BITS, pdTRUE, pdFALSE, portMAX_DELAY);
        if (ev & SW_1_SHORT_EV)
        {
            // 短按事件
            printf("GPIO[%d]: short pressed\n", BTN_GPIO_1);
        }
        else if(ev & SW_1_LONG_EV)
        {
            // 长按事件
            printf("GPIO[%d]: long pressed\n", BTN_GPIO_1);
        }
        else if (ev & SW_2_SHORT_EV)
        {
            // 短按事件
            printf("GPIO[%d]: short pressed\n", BTN_GPIO_2);
        }
        else if(ev & SW_2_LONG_EV)
        {
            // 长按事件
            printf("GPIO[%d]: long pressed\n", BTN_GPIO_2);
        }
        xEventGroupClearBits(s_pressEvent, SW_EV_BITS);
    }
}



/** 完整的按键程序
 * @param 无
 * @return 无
 */
void task_btn_test_queue(void *arg)
{
    queue_handle = xQueueCreate(10, sizeof(uint32_t));
    queue_data_t data;
    button_config_t btn_cfg =
        {
            .gpio_num = BTN_GPIO_1,         // gpio号
            .active_level = 0,              // 按下的电平
            .long_press_time = 3000,        // 长按时间
            .short_cb = short_press_handle, // 短按回调函数
            .long_cb = long_press_handle,   // 长按回调函数
            .arg = (void *)BTN_GPIO_1,
        };
    button_event_set(&btn_cfg); // 添加按键响应事件处理
    btn_cfg.gpio_num = BTN_GPIO_2;
    btn_cfg.arg = (void *)BTN_GPIO_2;
    button_event_set(&btn_cfg); // 添加按键响应事件处理
    while (1)
    {
        // 等待按键按下事件
        if(xQueueReceive(queue_handle, &data, portMAX_DELAY))
        {
            if (!(data.type))
            {
                // 短按事件
                printf("GPIO[%d]: short pressed\n", data.io_num);
            }
            else
            {
                // 长按事件
                printf("GPIO[%d]: long pressed\n", data.io_num);
            }
        }
    }
}
