
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "servo_motor.h"
#include "gpio_pin_config.h"

/* ***************************************************************************************************************** */
/*                                               macro define                                                        */
/* ***************************************************************************************************************** */
// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -90         // Minimum angle
#define SERVO_MAX_DEGREE 90          // Maximum angle

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

#define SERVO_STRUM_UP_ANGLE 20
#define SERVO_STRUM_DOWN_ANGLE -35

#ifdef CONFIG_SERVO_FRET
#define SERVO_FRET_UP_ANGLE 70
#define SERVO_FRET_DOWN_ANGLE 85
#endif

#define SERVO_SPEED 0.12 // s/60degree
/* ***************************************************************************************************************** */
/*                                               struct define                                                       */
/* ***************************************************************************************************************** */

/* ***************************************************************************************************************** */
/*                                               global variable                                                     */
/* ***************************************************************************************************************** */
static const char *TAG = "servo_motor";
TaskHandle_t g_servo_task_handle = NULL;
mcpwm_cmpr_handle_t g_strum_pwm_cmp = NULL;
mcpwm_cmpr_handle_t g_fret_pwm_cmp = NULL;
int g_strum_angle = SERVO_STRUM_UP_ANGLE;



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

void servo_motor_action(int index)
{
    switch (index) {
#ifdef CONFIG_SERVO_FRET
    case 1 :
        /* Release */
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_fret_pwm_cmp, angle_to_compare(SERVO_FRET_UP_ANGLE - 30)));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_fret_pwm_cmp, angle_to_compare(SERVO_FRET_UP_ANGLE)));
        break;
    case 2 :
        /* Fret */
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_fret_pwm_cmp, angle_to_compare(SERVO_FRET_DOWN_ANGLE -30)));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_fret_pwm_cmp, angle_to_compare(SERVO_FRET_DOWN_ANGLE)));
        break;
#endif
    case 3 :
        /* Strum */
        g_strum_angle = (g_strum_angle == SERVO_STRUM_UP_ANGLE) ? SERVO_STRUM_DOWN_ANGLE : SERVO_STRUM_UP_ANGLE;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_strum_pwm_cmp, angle_to_compare(g_strum_angle)));
        break;
    default:
        ESP_LOGE(TAG, "Invalid index: %d", index);
        break;
    }

    return;
}

void servo_motor_init(void)
{
    g_strum_pwm_cmp = pwm_create(SERVO_STRUM_GPIO);
    g_strum_angle = SERVO_STRUM_UP_ANGLE;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_strum_pwm_cmp, angle_to_compare(SERVO_STRUM_UP_ANGLE)));

#ifdef CONFIG_SERVO_FRET
    g_fret_pwm_cmp = pwm_create(SERVO_FRET_GPIO);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_fret_pwm_cmp, angle_to_compare(SERVO_FRET_UP_ANGLE)));
#endif // CONFIG_SERVO_FRET


    vTaskDelay(500 / portTICK_PERIOD_MS);   // wait for servo motor action to finish

    return;
}
