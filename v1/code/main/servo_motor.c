
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "servo_motor.h"
#include "gpio_pin_config.h"
#include "nvs_flash.h"

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

// #define SERVO_STRUM_UP_ANGLE 25
// #define SERVO_STRUM_DOWN_ANGLE -30
#define SERVO_STRUM_ANGLE 28

#ifdef CONFIG_HW_PROTOTYPE
#define SERVO_FRET_UP_ANGLE 70
#define SERVO_FRET_DOWN_ANGLE 85
#endif

#define SERVO_SPEED 0.12 // s/60degree

#define STORAGE_NAMESPACE "servo_motor"
/* ***************************************************************************************************************** */
/*                                               struct define                                                       */
/* ***************************************************************************************************************** */
typedef struct {
    struct {
        char *name;
        int output_gpio;
        int polarity;
        int init_angle;
    } const cfg;
    int mid_angle;
    mcpwm_cmpr_handle_t pwm_handle;
    int curr_angle;
} servo_t;
/* ***************************************************************************************************************** */
/*                                               global variable                                                     */
/* ***************************************************************************************************************** */
static const char *TAG = "servo_motor";
TaskHandle_t g_servo_task_handle = NULL;

static servo_t g_servo[] = {
    { // strum servo
        .cfg = {
            .name = "strum",
            .output_gpio = SERVO_STRUM_GPIO,
            .polarity = 1,
            .init_angle = SERVO_STRUM_ANGLE,
        },
        .pwm_handle = NULL,
        .mid_angle = 0,
        .curr_angle = 0,
    },
#ifdef CONFIG_HW_PROTOTYPE
    { // fret servo
        .cfg = {
            .name = "fret",
            .output_gpio = SERVO_FRET_GPIO,
            .polarity = 1,
            .init_angle = SERVO_FRET_UP_ANGLE,
        },
        .pwm_handle = NULL,
        .mid_angle = 0xffffffff,
        .curr_angle = SERVO_FRET_UP_ANGLE,
    },
#endif
};



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


void servo_motor_action(int act_idx)
{
    switch (act_idx) {
#ifdef CONFIG_HW_PROTOTYPE
    case 1 :
        /* Release */
        servo_set_angle(1, SERVO_FRET_UP_ANGLE);
        break;
    case 2 :
        /* Fret */
        servo_set_angle(1, SERVO_FRET_DOWN_ANGLE);
        break;
#endif
    case 3 :
        /* Strum */
        if (g_servo[0].curr_angle >= 0) {
            servo_set_angle(0, -SERVO_STRUM_ANGLE);
        } else {
            servo_set_angle(0, SERVO_STRUM_ANGLE);
        }
        break;
    default:
        ESP_LOGE(TAG, "Invalid action index: %d", act_idx);
        break;
    }

    return;
}

static int param_check(int servo_idx, int angle)
{
    if (servo_idx < 0 || servo_idx >= sizeof(g_servo) / sizeof(g_servo[0])) {
        ESP_LOGE(TAG, "Invalid servo index: %d", servo_idx);
        return ESP_ERR_INVALID_ARG;
    }
    if (angle < SERVO_MIN_DEGREE || angle > SERVO_MAX_DEGREE) {
        ESP_LOGE(TAG, "Invalid servo angle: %d", angle);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

int servo_set_angle(int servo_idx, int angle)
{
    int rc = param_check(servo_idx, angle);
    if (rc != ESP_OK) {
        return rc;
    }
    int real_angle = angle + g_servo[servo_idx].mid_angle;
    if (real_angle < SERVO_MIN_DEGREE || real_angle > SERVO_MAX_DEGREE) {
        ESP_LOGE(TAG, "angle %d + middle %d = %d, is out of range [%d, %d]", angle, g_servo[servo_idx].mid_angle, real_angle, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE);
        return ESP_ERR_INVALID_ARG;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_servo[servo_idx].pwm_handle, angle_to_compare(real_angle)));
    g_servo[servo_idx].curr_angle = angle;

    return ESP_OK;
}

void servo_motor_init(void)
{
    /* get middle angle from NVS */
    // Open
    nvs_handle_t nvs_handle;
    int rc = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle, use default angle 0", esp_err_to_name(rc));
    } else {
        for (int i = 0; i < sizeof(g_servo) / sizeof(g_servo[0]); i++) {
            // check if the middle angle is exists
            int32_t angle = 0;
            rc = nvs_get_i32(nvs_handle, g_servo->cfg.name, &angle);
            if (rc != ESP_OK) {
                if (rc == ESP_ERR_NVS_NOT_FOUND) {
                    /* nvs not exist */
                    rc = ESP_OK;
                    ESP_LOGW(TAG, "servo middle angle not found in NVS, use default angle 0");
                } else {
                    // Other error
                    ESP_LOGE(TAG, "Error (%s) reading NVS, use default angle 0", esp_err_to_name(rc));
                }
            } else {
                // nvs exist
                g_servo[i].mid_angle = angle;
                ESP_LOGI(TAG, "g_servo[%d].mid_angle = %d, loaded from NVS", i, g_servo[i].mid_angle);
            }
        }
    }
    nvs_close(nvs_handle);


    /* init pwm and angle */
    for (int i = 0; i < sizeof(g_servo) / sizeof(g_servo[0]); i++) {
        g_servo[i].pwm_handle = pwm_create(g_servo[i].cfg.output_gpio);
        int init_angle = g_servo[i].cfg.init_angle;
        servo_set_angle(i, init_angle);
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);   // wait for servo motor action to finish

    return;
}

int servo_set_middle_angle(int servo_idx, int mid_angle)
{
    int rc = param_check(servo_idx, mid_angle);
    if (rc != ESP_OK) {
        return rc;
    }
    /* set */
    g_servo[servo_idx].mid_angle = mid_angle;
    servo_set_angle(servo_idx, g_servo[servo_idx].curr_angle); // reset positon


    /* write to nvs */
    nvs_handle_t nvs_handle;
    rc = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(rc));
    } else {
        for (int i = 0; i < sizeof(g_servo) / sizeof(g_servo[0]); i++) {
            rc = nvs_set_i32(nvs_handle, g_servo->cfg.name, (int32_t)mid_angle);
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "Error (%s) writing NVS, It is only effective before reboot", esp_err_to_name(rc));
            }
        }
    }
    nvs_close(nvs_handle);


    ESP_LOGI(TAG, "Set servo[%d] middle angle to %d", servo_idx, mid_angle);

    return ESP_OK;
}
