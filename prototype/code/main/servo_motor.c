
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"


/* ***************************************************************************************************************** */
/*                                                 宏定义                                                            */
/* ***************************************************************************************************************** */
// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle


#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms


#define SERVO_STRUM_GPIO             15 // GPIO number for strum servo
#define SERVO_FRET_GPIO              16 // GPIO number for fret servo
/* ***************************************************************************************************************** */
/*                                                 结构体定义                                                         */
/* ***************************************************************************************************************** */

/* ***************************************************************************************************************** */
/*                                                 全局变量                                                           */
/* ***************************************************************************************************************** */
static const char *TAG = "example";
TaskHandle_t g_servo_task_handle = NULL;



static inline uint32_t angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

mcpwm_cmpr_handle_t pwm_create(int output_gpio)
{
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = output_gpio,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    return comparator;
}

void servo_motor_task(void *arg)
{
    mcpwm_cmpr_handle_t strum_pwm_cmp = pwm_create(SERVO_STRUM_GPIO);
    mcpwm_cmpr_handle_t fret_pwm_cmp = pwm_create(SERVO_FRET_GPIO);

    int angle = -20;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(fret_pwm_cmp, angle_to_compare(0))); 
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(strum_pwm_cmp, angle_to_compare(angle)));
    while (1) {
        // wait for angle
        int index;
        BaseType_t rc = xTaskNotifyWait(0, 0, &index, pdMS_TO_TICKS(1000));
        if (rc == pdFALSE) {
            continue;
        }

        switch (index) {
        case 1 :
            /* 抬起 */
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(fret_pwm_cmp, angle_to_compare(70)));  
            break;
        case 2 :
            /* 按下 */
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(fret_pwm_cmp, angle_to_compare(85)));  
            break;
        case 3 :
            /* 拨弦 */
            angle = -angle;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(strum_pwm_cmp, angle_to_compare(angle)));
            break;
        default:
            ESP_LOGE(TAG, "Invalid index: %d", index);
            break;
        } 
    }
}

void servo_motor_init(void)
{
    xTaskCreate(servo_motor_task, "servo_motor_task", 1024 * 10, NULL, 6, &g_servo_task_handle);
}

TaskHandle_t get_servo_motor_task_handle(void)
{
    return g_servo_task_handle;
}
