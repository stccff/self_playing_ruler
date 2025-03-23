/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"
#include "math.h"
#include "step_motor.h"

/* ***************************************************************************************************************** */
/*                                                 宏定义                                                            */
/* ***************************************************************************************************************** */
#define STEP_MOTOR_GPIO_EN 0
#define STEP_MOTOR_GPIO_DIR 2
#define STEP_MOTOR_GPIO_STEP 4

#define STEP_MOTOR_MODE0_PIN 40
#define STEP_MOTOR_MODE1_PIN 41
#define STEP_MOTOR_MODE2_PIN 42

#define STEP_MOTOR_ENABLE_LEVEL 0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

#define MODE 3 // 0: full step, 1: half step, 2: 1/4 step, 3: 1/8 step, 4: 1/16 step, 5: 1/32 step
#define SAMPLE_POINTS 500
#define SPEED_HZ (800 * pow(2, MODE))
#define MAX_FULL_STEP 300

/* ***************************************************************************************************************** */
/*                                                 结构体定义                                                         */
/* ***************************************************************************************************************** */

/* ***************************************************************************************************************** */
/*                                                 全局变量                                                           */
/* ***************************************************************************************************************** */
static const char *TAG = "step_motor";
TaskHandle_t g_step_motor_task_handle = NULL;


void step_motor_task(void *arg)
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
    rmt_channel_handle_t motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    ESP_LOGI(TAG, "Set spin direction");
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_LOGI(TAG, "Enable step motor");
    gpio_set_level(STEP_MOTOR_GPIO_EN, STEP_MOTOR_ENABLE_LEVEL);

    ESP_LOGI(TAG, "Set step motor mode: %d", MODE);
    gpio_set_level(STEP_MOTOR_MODE0_PIN, MODE & 0x01);
    gpio_set_level(STEP_MOTOR_MODE1_PIN, (MODE & 0x02) >> 1);
    gpio_set_level(STEP_MOTOR_MODE2_PIN, (MODE & 0x04) >> 2);

    ESP_LOGI(TAG, "Create motor encoders");
    // stepper_motor_curve_encoder_config_t accel_encoder_config = {
    //     .resolution = STEP_MOTOR_RESOLUTION_HZ,
    //     .sample_points = SAMPLE_POINTS,
    //     .start_freq_hz = 500,
    //     .end_freq_hz = SPEED_HZ,
    // };
    // rmt_encoder_handle_t accel_motor_encoder = NULL;
    // ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    // stepper_motor_curve_encoder_config_t decel_encoder_config = {
    //     .resolution = STEP_MOTOR_RESOLUTION_HZ,
    //     .sample_points = SAMPLE_POINTS,
    //     .start_freq_hz = SPEED_HZ,
    //     .end_freq_hz = 500,
    // };
    // rmt_encoder_handle_t decel_motor_encoder = NULL;
    // ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan));

    ESP_LOGI(TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    const static uint32_t accel_samples = SAMPLE_POINTS;
    const static uint32_t uniform_speed_hz = SPEED_HZ;
    const static uint32_t decel_samples = SAMPLE_POINTS;

    /* 位置归零 */
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE); // set direction
    tx_config.loop_count = MAX_FULL_STEP * pow(2, MODE);
    ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    gpio_set_level(STEP_MOTOR_GPIO_EN, 1); // disable motor

    xTaskNotify(get_step_motor_task_handle(), (uint32_t)150, eSetValueWithOverwrite);
    vTaskDelay(100 / portTICK_PERIOD_MS);


    int pos = 0;
    int curr_pos = 0;
    int step = 0;
    int last_dir = STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE;
    while (1) {
        // wait for direction and step
        
        BaseType_t rc = xTaskNotifyWait(0, 0, &pos, pdMS_TO_TICKS(1000));
        if (rc == pdFALSE) {
            continue;
        }
        if (pos > MAX_FULL_STEP || pos < 0) {
            ESP_LOGE(TAG, "Invalid step: %d", step);
            continue;
        }
        step = pos - curr_pos;
        curr_pos = pos;
        tx_config.loop_count = abs(step) * pow(2, MODE);
        gpio_set_level(STEP_MOTOR_GPIO_EN, 0); // enable motor
        int dir = (step > 0) ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE;
        // 如方向改变，旷量补偿
        if (last_dir != dir && step != 0) {
            tx_config.loop_count += 2 * pow(2, MODE);
        }
        last_dir = dir;
        gpio_set_level(STEP_MOTOR_GPIO_DIR, dir); // set direction
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(STEP_MOTOR_GPIO_EN, 1); // disable motor
    }
}

void step_motor_init(void)
{
    xTaskCreate(step_motor_task, "step_motor_task", 1024 * 10, NULL, 9, &g_step_motor_task_handle);
}

TaskHandle_t get_step_motor_task_handle(void)
{
    return g_step_motor_task_handle;
}

