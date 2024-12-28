#ifndef _BUTTON_H_
#define _BUTTON_H_
#include "esp_err.h"

//按键回调函数
typedef void(*button_press_cb)(void * arg);

//按键配置结构体
typedef struct
{
    int gpio_num;           //gpio号
    int active_level;       //按下的电平
    int long_press_time;    //长按时间
    int short_press_time;   //短按时间
    button_press_cb short_cb;   //短按回调函数
    button_press_cb long_cb;    //长按回调函数
    void * arg;
}button_config_t;

/** 设置按键事件
 * @param cfg   配置结构体
 * @return ESP_OK or ESP_FAIL 
*/
esp_err_t button_event_set(button_config_t *cfg);

void task_btn_test(void* arg);
void task_btn_test_queue(void* arg);

#endif
