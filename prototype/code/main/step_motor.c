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

#define POW(x, y) ((y == 0) ? 1 : pow(x, y))

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

#define MODE 0 // 0: full step, 1: half step, 2: 1/4 step, 3: 1/8 step, 4: 1/16 step, 5: 1/32 step
#define SAMPLE_POINTS 500
#define SPEED_HZ (500 * POW(2, MODE))

#define LEN_PER_FULL_STEP 0.15 // mm
#define MAX_FULL_STEP 296
#define LEN_PER_STEP (LEN_PER_FULL_STEP / POW(2, MODE))
#define MAX_STEP (MAX_FULL_STEP * POW(2, MODE))
#define LEAD_SCREW_BACKLASH (2 * POW(2, MODE)) // step

// ruler features
#define RULER_LENGHT_MIN 13.5   // mm 13.7?
#define RULER_LENGTH_MAX 58     // mm

#define RULER_FREQ_MIN 73.42    // ≈55.80mm
#define RULER_FREQ_MAX 349.23   // ≈25.61mm

/* formula f=k/(L^2), L=(k/f)^(1/2) */
#define k 229049.0652


/* ***************************************************************************************************************** */
/*                                                 结构体定义                                                         */
/* ***************************************************************************************************************** */

/* ***************************************************************************************************************** */
/*                                                 全局变量                                                           */
/* ***************************************************************************************************************** */
static const char *TAG = "step_motor";
TaskHandle_t g_step_motor_task_handle = NULL;
int g_curr_pos;
rmt_channel_handle_t g_motor_chan = NULL;
/* ***************************************************************************************************************** */
/*                                           function prototype                                                      */
/* ***************************************************************************************************************** */

static double convert_len_to_freq(double len)
{
    return k / POW(len, 2);
}

static double convert_freq_to_len(double freq)
{
    return sqrt(k / freq);
}

static int convert_len_to_pos(double len)
{
    if (len < RULER_LENGHT_MIN || len > RULER_LENGTH_MAX) {
        ESP_LOGE(TAG, "Invalid length: %f", len);
        return -1;
    }
    return (int)((len - RULER_LENGHT_MIN) / (RULER_LENGTH_MAX - RULER_LENGHT_MIN) * MAX_STEP);
}

static int convert_len_to_step(double len)
{
    int pos = convert_len_to_pos(len);
    if (pos < 0) {
        return 0; // do not set step
    }
    int step = pos - g_curr_pos;
    g_curr_pos = pos;
    return step;
}

// static int convert_note_to_pos(char *note)
// {
//     // note-->midi

//     // midi-->frequency
//     // double freq = g_midi_freq[midi];



//     //frequency-->length
//     double length = sqrt(k / freq);
//     ESP_LOGI(TAG, "midi =%d, freq = %f, length = %f", midi, freq, length);
//     //length-->pos
//     if (length < LENGHT_MIN) {
//         ESP_ERROR_CHECK(ESP_FAIL);
//     }
//     int pos = (int)((length - LENGHT_MIN) / TOTAL_LENGTH * TOTAL_POS);

//     return pos;
// }



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
    ESP_ERROR_CHECK(rmt_enable(g_motor_chan));

    ESP_LOGI(TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    const static uint32_t accel_samples = SAMPLE_POINTS;
    const static uint32_t uniform_speed_hz = SPEED_HZ;
    const static uint32_t decel_samples = SAMPLE_POINTS;

    /* 位置归零 */
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE); // set direction
    tx_config.loop_count = MAX_STEP + 10 * POW(2, MODE);
    ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(10 / portTICK_PERIOD_MS);


    tx_config.loop_count = MAX_STEP / 2;
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    gpio_set_level(STEP_MOTOR_GPIO_EN, 1); // disable motor

    // init current position
    g_curr_pos = MAX_STEP / 2;

    int last_dir = STEP_MOTOR_SPIN_DIR_CLOCKWISE;
    while (1) {
        /* wait for receiving frequency */
        float freq;
        BaseType_t rc = xTaskNotifyWait(0, 0, (uint32_t*)&freq, pdMS_TO_TICKS(1000));
        if (rc == pdFALSE) {
            continue;
        }
        if (freq < RULER_FREQ_MIN || freq > RULER_FREQ_MAX) {
            ESP_LOGE(TAG, "Invalid frequency: %f", freq);
            continue;
        }
        /* get motor step */
        // freq --> length
        double len = convert_freq_to_len(freq);
        // absolute length --> relative step
        int step = convert_len_to_step(len);
        if (step == 0) {
            continue;
        }
        tx_config.loop_count = abs(step);
        gpio_set_level(STEP_MOTOR_GPIO_EN, 0); // enable motor
        int dir = (step > 0) ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE;
        // If the direction changes, compensate for backlash
        if (last_dir != dir) {
            tx_config.loop_count += LEAD_SCREW_BACKLASH;
        }
        last_dir = dir;
        gpio_set_level(STEP_MOTOR_GPIO_DIR, dir); // set direction
        ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
        ESP_LOGI(TAG, "step = %d, freq = %f, len = %f", step, freq, len);
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
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

void wait_motor_done(void)
{
    rmt_tx_wait_all_done(g_motor_chan, -1);
}

