#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "servo_motor.h"
#include "step_motor.h"
#include "ruler.h"
#include "uart_ctrl.h"
#include "digital_mic.h"
#include "h_bridge.h"
#include "tinyusb_device.h"
#include "play.h"
#include "nvs_flash.h"
#include "key.h"

#define TAG "MAIN"

/*
 vTaskGetRunTimeStats() 使用
 注意:
 使用 vTaskGetRunTimeStats() 前需使能:
 make menuconfig -> Component config -> FreeRTOS -> configUSE_TRACE_FACILITY
 make menuconfig -> Component config -> FreeRTOS -> Enable FreeRTOS trace facility -> configUSE_STATS_FORMATTING_FUNCTIONS
 make menuconfig -> Component config -> FreeRTOS -> Enable display of xCorelD in vlaskList
 make menuconfig -> Component config -> FreeRTOS -> configGENERATE_RUN_TIME_STATS
 通过上面配置，等同于使能 FreeRTOSConfig.h 中如下三个宏:
 configGENERATE_RUN_TIME_STATS，configUSE_STATS_FORMATTING_FUNCTIONS 和 configSUPPORT_DYNAMIC_ALLOCATION
 */
void print_task_info(char *buff)
{
    /* 打印当前任务列表 */
    vTaskList(buff);
    printf("task\t\tstate\tprio\tstack\ttid\tcore\n");
    printf("%s\n", buff);
    /* 打印任务运行信息 */
    vTaskGetRunTimeStats(buff);
    printf("task_name\trun_cnt\t\tusage\n");
    printf("%s\n", buff);
}

void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    h_bridge_init();
    init_nvs();
    servo_motor_init();
    stepper_motor_init();
    ruler_init();
    i2s_driver_init();
    play_init();
    uart_init();
    button_init();
    tinyusb_device_init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    char *buff = (char *)malloc(2048);
    print_task_info(buff);
    free(buff);

    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    return;
}
