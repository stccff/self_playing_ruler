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
#include "uart_ctrl.h"
#include "digital_mic.h"
#include "electromagnet.h"



void app_main(void)
{
    i2s_driver_init();
    electromagnet_init();
    servo_motor_init();
    init_nvs_for_freq_table();
    stepper_motor_init();
    uart_init();

    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    return;
}
