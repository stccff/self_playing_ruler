
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "servo_motor.h"
#include "gpio_pin_config.h"
#include "nvs_flash.h"
#include <math.h>
#include "play.h"
#include "step_motor.h"
#include "digital_mic.h"

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

#define SAMPLE_RATE 8000 // Hz
#define MAX_CALIBRATE_TIMES 25
#define ONECE_SAMPLE_TIMES 3
#define CALI_SAMPLE_TIME 300 // ms
#define CALI_SAMPLE_NUM (CALI_SAMPLE_TIME * SAMPLE_RATE / 1000) // TODO: if change, make sure dma buffer is enough
#define I2S_BIT_WIDTH 24 // TODO: do not define again
#define THRESHOLD_DELTA_PERCENTAGE 0.015 // 1.5%

#define STORAGE_NAMESPACE "servo_motor"
/* ***************************************************************************************************************** */
/*                                               struct define                                                       */
/* ***************************************************************************************************************** */
typedef struct {
    struct {
        char *name;
        int output_gpio;
        int polarity;
        float init_angle;
    } const cfg;
    float offset_angle;
    int offset_angle_r;
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
        .offset_angle = 0,
        .offset_angle_r = 0,
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
        .offset_angle = 0xfffffffg,
        .offset_angle_r = 0xfffffffg,
        .curr_angle = SERVO_FRET_UP_ANGLE,
    },
#endif
};



static inline uint32_t angle_to_compare(float angle)
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

/**
 * @brief this is an IRAM function and not a common API, it is only used for strum action callback function,
 *          which has a high requirement for real-time performance.
 *
 */
void IRAM_ATTR servo_strum_iram_without_fpu(void)
{
    /* Strum */
    int angle = 0;
    if (g_servo[0].curr_angle >= 0) {
        angle = -SERVO_STRUM_ANGLE;
    } else {
        angle = SERVO_STRUM_ANGLE;
    }

    int real_angle = angle + g_servo[0].offset_angle_r;
    ESP_ERROR_CHECK(real_angle < SERVO_MIN_DEGREE || real_angle > SERVO_MAX_DEGREE);
    uint32_t cmp_val = (real_angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
                        (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_servo[0].pwm_handle, cmp_val));
    g_servo[0].curr_angle = angle;
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

static int param_check(int servo_idx, float angle)
{
    if (servo_idx < 0 || servo_idx >= sizeof(g_servo) / sizeof(g_servo[0])) {
        ESP_LOGE(TAG, "Invalid servo index: %d", servo_idx);
        return ESP_ERR_INVALID_ARG;
    }
    if (angle < SERVO_MIN_DEGREE || angle > SERVO_MAX_DEGREE) {
        ESP_LOGE(TAG, "Invalid servo angle: %f", angle);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

int servo_set_angle(int servo_idx, float angle)
{
    int rc = param_check(servo_idx, angle);
    if (rc != ESP_OK) {
        return rc;
    }
    float real_angle = angle + g_servo[servo_idx].offset_angle;
    if (real_angle < SERVO_MIN_DEGREE || real_angle > SERVO_MAX_DEGREE) {
        ESP_LOGE(TAG, "angle %f + offset %f = %f, is out of range [%d, %d]", angle, g_servo[servo_idx].offset_angle, real_angle, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE);
        return ESP_ERR_INVALID_ARG;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_servo[servo_idx].pwm_handle, angle_to_compare(real_angle)));
    g_servo[servo_idx].curr_angle = angle;

    return ESP_OK;
}


int servo_get_curr_angle(int servo_idx, float *angle)
{
    int rc = param_check(servo_idx, 0);
    if (rc != ESP_OK) {
        return rc;
    }
    if (angle == NULL) {
        ESP_LOGE(TAG, "Invalid angle pointer");
        return ESP_ERR_INVALID_ARG;
    }
    *angle = g_servo[servo_idx].curr_angle;
    return ESP_OK;
}

void servo_motor_init(void)
{
    /* get offset angle from NVS */
    // Open
    nvs_handle_t nvs_handle;
    int rc = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle, use default angle 0", esp_err_to_name(rc));
    } else {
        for (int i = 0; i < sizeof(g_servo) / sizeof(g_servo[0]); i++) {
            // check if the offset angle is exists
            int32_t angle = 0;
            rc = nvs_get_i32(nvs_handle, g_servo->cfg.name, &angle);
            if (rc != ESP_OK) {
                if (rc == ESP_ERR_NVS_NOT_FOUND) {
                    /* nvs not exist */
                    rc = ESP_OK;
                    ESP_LOGW(TAG, "servo offset angle not found in NVS, use default angle 0");
                } else {
                    // Other error
                    ESP_LOGE(TAG, "Error (%s) reading NVS, use default angle 0", esp_err_to_name(rc));
                }
            } else {
                // nvs exist
                g_servo[i].offset_angle = angle;
                g_servo[i].offset_angle_r = round(angle);
                ESP_LOGI(TAG, "g_servo[%d].offset_angle = %f, loaded from NVS", i, g_servo[i].offset_angle);
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

int servo_set_offset_angle(int servo_idx, float offset_angle)
{
    int rc = param_check(servo_idx, offset_angle);
    if (rc != ESP_OK) {
        return rc;
    }
    /* set */
    g_servo[servo_idx].offset_angle = offset_angle;
    g_servo[servo_idx].offset_angle_r = round(offset_angle);
    servo_set_angle(servo_idx, g_servo[servo_idx].curr_angle); // reset positon


    /* write to nvs */
    nvs_handle_t nvs_handle;
    rc = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(rc));
    } else {
        for (int i = 0; i < sizeof(g_servo) / sizeof(g_servo[0]); i++) {
            rc = nvs_set_i32(nvs_handle, g_servo->cfg.name, (int32_t)round(offset_angle));
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "Error (%s) writing NVS, It is only effective before reboot", esp_err_to_name(rc));
            }
        }
    }
    nvs_close(nvs_handle);


    ESP_LOGI(TAG, "Set servo[%d] offset angle to %f, and %d to nvs", servo_idx, offset_angle, (int)round(offset_angle));

    return ESP_OK;
}

/**
 * @brief find the max peak in the data, and calculate the latency
 *
 * @param data
 * @param len
 * @return float
 */
static float calc_latency(float *data, size_t len)
{
    int idx[3] = {-1, -1, -1};
    float val[3] = {0, 0, 0}; // max--min

    for (size_t i = 0; i < len; i++) {
        float abs_val = fabs(data[i]);
        if (abs_val > val[0]) {
            val[2] = val[1]; idx[2] = idx[1];
            val[1] = val[0]; idx[1] = idx[0];
            val[0] = abs_val; idx[0] = i;
        } else if (abs_val > val[1]) {
            val[2] = val[1]; idx[2] = idx[1];
            val[1] = abs_val; idx[1] = i;
        } else if (abs_val > val[2]) {
            val[2] = abs_val; idx[2] = i;
        }
    }

    // printf("Top 3 indices: %d, %d, %d\n", idx[0], idx[1], idx[2]);
    int index = round((idx[0] + idx[1] + idx[2]) / 3.0);

    return (float)index / SAMPLE_RATE;
}


int servo_offset_calibration(void)
{
    int rc = ESP_OK;
    rc = servo_set_offset_angle(0, 0); // reset offset before do calibration
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "servo calibration: set servo offset angle fail");
        return rc;
    }

    // enable mic
    mic_reconfig_sample_rate(SAMPLE_RATE);
    mic_enable(true);


    rc = play_single_note_by_pos(MAX_STEP / 2); // move to middle pos
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "servo calibration: move to middle pos fail");
        return rc;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    float angle = 0;
    servo_get_curr_angle(0, &angle);
    int init_dir = (angle < 0) ? 1 : -1;

    uint8_t *i2s_buff = (uint8_t *)malloc(CALI_SAMPLE_NUM * I2S_BIT_WIDTH / 8);
    float *out_buff = (float *)malloc(CALI_SAMPLE_NUM * sizeof(float));
    float latency[ONECE_SAMPLE_TIMES*2] = {0};
    int counter = 0;
    float offset_angle = 0;
    int cali_num = 0;

    for (cali_num = 0; cali_num < MAX_CALIBRATE_TIMES; cali_num++) {
        for (int i = 0; i < ONECE_SAMPLE_TIMES*2; i++) {
            // clear mic buffer
            rc = read_mic_data(NULL, NULL, 0, true);
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "clear mic buff fail, with error: %d", rc);
                goto err;
            }
            // strum
            rc = play_single_note_by_pos(MAX_STEP / 2);
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "play by pos fail");
                goto err;
            }
            // record
            rc = read_mic_data(i2s_buff, out_buff, CALI_SAMPLE_NUM, false);
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "read mic data fail, with error: %d", rc);
                goto err;
            }
            // calculate latency of strum
            latency[i] = calc_latency(out_buff, CALI_SAMPLE_NUM);
            vTaskDelay(pdMS_TO_TICKS(10)); // release cpu
        }


        float delta = 0;
        float sum = 0;
        for (int i = 0; i < ONECE_SAMPLE_TIMES; i++) {
            delta += latency[i*2] - latency[i*2 + 1];
            sum += latency[i*2] + latency[i*2 + 1];
        }
        float percent = delta / sum;
        ESP_LOGI(TAG, "servo calibration: latency[0] = %f, latency[1] = %f, percent = %f", latency[0], latency[1], percent);
        // If error is small enough and continuous 3 times, then finish calibration
        if (fabs(percent) <= THRESHOLD_DELTA_PERCENTAGE) {
            counter++;
            if (counter >= 3) {
                ESP_LOGI(TAG, "servo calibration success, %d times, offset angle = %d", cali_num, (int)round(offset_angle));
                break;
            }
            continue;
        }

        counter = 0; // reset counter

        // do compensation
        offset_angle += init_dir * percent * 20; // this is a proportional control, 20 is a gain
        rc = servo_set_offset_angle(0, offset_angle);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "servo calibration: set servo offset angle fail");
            (void)servo_set_offset_angle(0, 0);
            goto err;
        }
    }

    if (cali_num >= MAX_CALIBRATE_TIMES) {
        ESP_LOGE(TAG, "servo calibration: fail, reach max calibration times: %d", MAX_CALIBRATE_TIMES);
        rc = ESP_FAIL;
        (void)servo_set_offset_angle(0, 0);
        goto err;
    }

err:
    mic_enable(false); // disable mic channel

    return rc;
}
