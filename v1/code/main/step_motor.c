/*
 * Copyright (c) 2025 by stccff
 *
 * This software is developed by an individual and is intended for learning and communication purposes only.
 * You are permitted to freely copy, modify, and distribute this code for non-commercial use,
 * provided that this copyright notice is retained.
 * For commercial use, please contact the author for authorization.
 *
 * Author Email: [913602792@qq.com]
 */
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"
#include "gpio_pin_config.h"
#include "h_bridge.h"
#include "step_motor.h"
/* ***************************************************************************************************************** */
/*                                                 macro define                                                      */
/* ***************************************************************************************************************** */

// hardware A v1.1
#ifdef CONFIG_HW_A_VER_1_1
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 1
#define SAMPLE_POINTS (20 * (1 << MODE))
#define SPEED_LOW_HZ (500 * (1 << MODE)) // < SPEED_HZ
#define SPEED_HZ (2000 * (1 << MODE))
#endif

#define STEP_MOTOR_ENABLE_LEVEL 0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

/* ***************************************************************************************************************** */
/*                                                 struct define                                                     */
/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */
/*                                                global variable                                                    */
/* ***************************************************************************************************************** */
static const char *TAG = "step_motor";
rmt_channel_handle_t g_motor_chan = NULL;
int g_curr_pos;
int g_last_dir;
rmt_transmit_config_t g_tx_config = {0};
rmt_encoder_handle_t g_uniform_motor_encoder = NULL;
rmt_encoder_handle_t g_accel_motor_encoder = NULL;
rmt_encoder_handle_t g_decel_motor_encoder = NULL;
/* ***************************************************************************************************************** */
/*                                           function prototype                                                      */
/* ***************************************************************************************************************** */

/**
 * @brief convert the absolute step to relative step
 *
 * @param pos absolute step
 * @return int
 */
static int convert_pos_to_step(int pos)
{
    int step = pos - g_curr_pos;
    g_curr_pos = pos;
    return step;
}

/**
 * @brief move stepper motor to specified position. stepper_motor_async_wait_done must be called, when use async mod
 *
 * @param is_sync sync/async mode
 * @param pos absolute step of stepper motor
 * @return int
 */
int stepper_motor_action_by_pos(bool is_sync, int pos) // TODO: why memory error occurred in rmt??? when I use async mode
{
#ifdef CONFIG_DEBUG_PRINT
        int64_t start = esp_timer_get_time();
#endif

    if (pos < 0 || pos > MAX_STEP) {
        ESP_LOGE(TAG, "invalid pos: %d", pos);
        return ESP_ERR_INVALID_ARG;
    }

    int step = convert_pos_to_step(pos);
    if (step == 0) { // do nothing
        return ESP_OK;
    }

    int dir = (step > 0) ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE;

    g_last_dir = dir;
    gpio_set_level(STEP_MOTOR_GPIO_DIR, dir); // set direction
    gpio_set_level(STEP_MOTOR_GPIO_EN, 0); // enable motor

    uint32_t step_num = abs(step);
    uint32_t samples;
    uint32_t accel_samples = SAMPLE_POINTS;
    uint32_t decel_samples = SAMPLE_POINTS;
    uint32_t uniform_speed_hz = SPEED_HZ;
    if (step_num <= SAMPLE_POINTS * 2) {
        // acceleration phase
        g_tx_config.loop_count = 0;
        samples = step_num / 2 + step_num % 2;
        ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_accel_motor_encoder, &samples, sizeof(samples), &g_tx_config));
        // deceleration phase
        g_tx_config.loop_count = 0;
        samples = step_num / 2;
        ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_decel_motor_encoder, &samples, sizeof(samples), &g_tx_config));
    } else {
        // acceleration phase
        g_tx_config.loop_count = 0;
        ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_accel_motor_encoder, &accel_samples, sizeof(accel_samples), &g_tx_config));
        // uniform phase
        g_tx_config.loop_count = step_num - accel_samples - decel_samples;
        ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &g_tx_config));
        // deceleration phase
        g_tx_config.loop_count = 0;
        ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_decel_motor_encoder, &decel_samples, sizeof(decel_samples), &g_tx_config));
    }

    if (is_sync) {
        // wait all transactions finished
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));

        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(STEP_MOTOR_GPIO_EN, 1); // disable motor
    }

#ifdef CONFIG_DEBUG_PRINT
        int64_t end = esp_timer_get_time();
        ESP_LOGI(TAG, "stepper motor action execution time: %lld ms", (end - start) / 1000);
#endif

    return ESP_OK;
}

/**
 * @brief wait for stepper motor move done
 *
 */
void stepper_motor_async_wait_done(void)
{
    // wait all transactions finished
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));

    vTaskDelay(pdMS_TO_TICKS(10)); //TODO: is need?
    gpio_set_level(STEP_MOTOR_GPIO_EN, 1); // disable motor
}

void stepper_motor_init(void)
{
    ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_EN |
                        1ULL << STEP_MOTOR_MODE0_PIN | 1ULL << STEP_MOTOR_MODE1_PIN | 1ULL << STEP_MOTOR_MODE2_PIN,

    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    ESP_LOGI(TAG, "Create RMT TX channel");
    // rmt_channel_handle_t g_motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &g_motor_chan));

    ESP_LOGI(TAG, "Set spin direction");
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_LOGI(TAG, "Enable step motor");
    gpio_set_level(STEP_MOTOR_GPIO_EN, STEP_MOTOR_ENABLE_LEVEL);

    ESP_LOGI(TAG, "Set step motor mode: %d", MODE);
    gpio_set_level(STEP_MOTOR_MODE0_PIN, MODE & 0x01);
    gpio_set_level(STEP_MOTOR_MODE1_PIN, (MODE & 0x02) >> 1);
    gpio_set_level(STEP_MOTOR_MODE2_PIN, (MODE & 0x04) >> 2);

    ESP_LOGI(TAG, "Create motor encoders");
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = SAMPLE_POINTS,
        .start_freq_hz = SPEED_LOW_HZ,
        .end_freq_hz = SPEED_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &g_accel_motor_encoder));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &g_uniform_motor_encoder));

    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = SAMPLE_POINTS,
        .start_freq_hz = SPEED_HZ,
        .end_freq_hz = SPEED_LOW_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &g_decel_motor_encoder));

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(g_motor_chan));

    /* Screw stepper motor init */
    ESP_LOGI(TAG, "Move to central position");

    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE); // len max dir
    g_tx_config.loop_count = MAX_STEP;
    uint32_t speed = SPEED_LOW_HZ;
    ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_uniform_motor_encoder, &speed, sizeof(speed), &g_tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(20 / portTICK_PERIOD_MS);

    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE); // len max dir
    g_tx_config.loop_count = MAX_STEP * 1.2;
    ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_uniform_motor_encoder, &speed, sizeof(speed), &g_tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(20 / portTICK_PERIOD_MS);

    g_tx_config.loop_count = MAX_STEP / 2;
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_uniform_motor_encoder, &speed, sizeof(speed), &g_tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(20 / portTICK_PERIOD_MS);

    gpio_set_level(STEP_MOTOR_GPIO_EN, 1); // disable motor

    // init current position
    g_curr_pos = MAX_STEP / 2;
    g_last_dir = STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE;
}

/**
 * @brief calculate the estimated time
 *
 * @return int >=0 ms
 *             <0 error
 */
int calc_stepper_motor_time_by_pos(int pos)
{
    if (pos < 0 || pos > MAX_STEP) {
        ESP_LOGE(TAG, "invalid pos: %d", pos);
        return -1;
    }

    float cost_time = 0;

    // T(p) = kp + b
    static const float k = -(1.0/SPEED_LOW_HZ - 1.0/SPEED_HZ) / (SAMPLE_POINTS - 1);
    static const float b = 1.0/SPEED_LOW_HZ;
    // T(n) = t1+t2+..+tn = n * (T(1) + T(n)) / 2 = n * (b + kn + b) / 2
#define TOTAL_TIME(n) ((n) * (b + k*(n) + b) / 2)

    int step_num = abs(pos - g_curr_pos);
    if (step_num == 0) {
        return 0;
    }

    if (step_num <= SAMPLE_POINTS * 2) {
        float acc_time = TOTAL_TIME(step_num);
        cost_time += acc_time * 2;
    } else {
        float acc_time = TOTAL_TIME(SAMPLE_POINTS);
        float flat_time = (step_num - SAMPLE_POINTS * 2) * (1.0/SPEED_HZ);
        cost_time += acc_time * 2 + flat_time;
    }

    return round(cost_time * 1000);
}
