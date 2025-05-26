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



void app_main(void)
{
    servo_motor_init();
    vTaskDelay(500 / portTICK_PERIOD_MS);   // wait for servo motor action to finish
    stepper_motor_init();
    uart_init();

    return;
}
