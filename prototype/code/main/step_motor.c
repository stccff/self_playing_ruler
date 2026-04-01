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

#define MODE 1 // 0: full step, 1: half step, 2: 1/4 step, 3: 1/8 step, 4: 1/16 step, 5: 1/32 step

#define SAMPLE_POINTS (20 * pow(2, MODE))
#define SPEED_LOW_HZ (500 * pow(2, MODE)) // < SPEED_HZ
#define SPEED_HZ (1200 * pow(2, MODE))

#define LEN_PER_FULL_STEP 0.15 // mm
#define MAX_FULL_STEP 296
#define LEN_PER_STEP (LEN_PER_FULL_STEP / POW(2, MODE))
#define MAX_STEP (MAX_FULL_STEP * POW(2, MODE))
#define LEAD_SCREW_BACKLASH (2 * POW(2, MODE)) // step

// ruler features
#define RULER_LENGHT_MIN 13.5   // mm 13.7?
#define RULER_LENGTH_MAX 58     // mm

// #define RULER_FREQ_MIN 73.42    // ≈55.80mm
// #define RULER_FREQ_MAX 349.23   // ≈25.61mm

/* formula f=k/(L^2), L=(k/f)^(1/2) */
#define k 194543.9295 // not ok


/* ***************************************************************************************************************** */
/*                                                 结构体定义                                                         */
/* ***************************************************************************************************************** */
struct LenFreqTlb {
    float len;
    float freq;
};
/* ***************************************************************************************************************** */
/*                                                 全局变量                                                           */
/* ***************************************************************************************************************** */
static const char *TAG = "step_motor";
TaskHandle_t g_step_motor_task_handle = NULL;
int g_curr_pos;
rmt_channel_handle_t g_motor_chan = NULL;
int g_last_dir;
rmt_transmit_config_t g_tx_config = {0};
rmt_encoder_handle_t g_uniform_motor_encoder = NULL;
rmt_encoder_handle_t g_accel_motor_encoder = NULL;
rmt_encoder_handle_t g_decel_motor_encoder = NULL;

struct LenFreqTlb g_lf_table[] = {
    {24.5, 305},
    {25, 293},
    {25.5, 285},
    {26, 268},
    {26.5, 262},
    {27, 250},
    {27.5, 248},
    {28, 237},
    {28.5, 235},
    {29, 229},
    {29.5, 227},
    {30, 220},
    {30.5, 214},
    {31, 208},
    {31.5, 200},
    {32, 192},
    {33, 185},
    {34, 172},
    {35, 165},
    {36, 157},
    {37, 148},
    {38, 144},
    {39, 137},
    {40, 133},
    {41, 126},
    {42, 121},
    {43, 114},
    {44, 108},
    {45, 104},
    {46, 98},
    {47, 93},
    {48, 90},
    {49, 87},
    {50, 83},
    {51, 81},
    {52, 80},
    {53, 78},
    {54, 77},
    {55, 76},
    {56, 75},
    {57, 75},
};
/* ***************************************************************************************************************** */
/*                                           function prototype                                                      */
/* ***************************************************************************************************************** */
static double get_len_by_freq(double target_freq) {
    int table_size = sizeof(g_lf_table) / sizeof(g_lf_table[0]);

    // 处理边界情况
    if (target_freq >= g_lf_table[0].freq) {
        ESP_LOGE(TAG, "freq:%lf in outof table, use:%f", target_freq, g_lf_table[0].freq);
        return g_lf_table[0].len;
    }
    if (target_freq <= g_lf_table[table_size-1].freq) {
        ESP_LOGE(TAG, "freq:%lf in outof table, use:%f", target_freq, g_lf_table[table_size-1].freq);
        return g_lf_table[table_size-1].len;
    }

    // 遍历查找相邻点进行插值
    for (int i = 0; i < table_size - 1; i++) {
        // 检查是否在当前区间内
        if (g_lf_table[i].freq >= target_freq && target_freq >= g_lf_table[i+1].freq) {
            float f1 = g_lf_table[i].freq;
            float l1 = g_lf_table[i].len;
            float f2 = g_lf_table[i+1].freq;
            float l2 = g_lf_table[i+1].len;

            // 线性插值计算
            float t = (target_freq - f1) / (f2 - f1);
            float interpolated_len = l1 + t * (l2 - l1);
            return interpolated_len;
        }
    }

    return -1.0f; // 理论上不会执行到这里
}

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

int step_motor_action_by_len(double len)
{
    // absolute length --> relative step
    int step = convert_len_to_step(len);
    if (step == 0) {
        return step;
    }
    int dir = (step > 0) ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE;
    // If the direction changes, compensate for backlash
    if (g_last_dir != dir) {
        g_tx_config.loop_count += LEAD_SCREW_BACKLASH;
    }
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
    // wait all transactions finished
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(STEP_MOTOR_GPIO_EN, 1); // disable motor

    return step;
}

/**
 * @brief stepper motor act as the input freq
 *
 * @param freq
 * @return int step numbers
 */
int step_motor_action(double freq)
{
    // if (freq < RULER_FREQ_MIN || freq > RULER_FREQ_MAX) {
    //     ESP_LOGE(TAG, "Invalid frequency: %f, no action", freq);
    //     return 0;
    // }
    // freq --> length
    double len = get_len_by_freq(freq);
    if (len < 0) {
        ESP_LOGI(TAG, "find table error");
        return 0;
    }
    int step = step_motor_action_by_len(len);
    ESP_LOGI(TAG, "freq = %f, len = %f, step = %d", freq, len, step);
    return step;
}

void step_motor_init(void)
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


    ESP_LOGI(TAG, "Move to central position");

    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    g_tx_config.loop_count = MAX_STEP + 10 * POW(2, MODE);
    uint32_t speed = SPEED_LOW_HZ;
    ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_uniform_motor_encoder, &speed, sizeof(speed), &g_tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
    g_tx_config.loop_count = MAX_STEP + 10 * POW(2, MODE);
    ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_uniform_motor_encoder, &speed, sizeof(speed), &g_tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    g_tx_config.loop_count = MAX_STEP / 2;
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_uniform_motor_encoder, &speed, sizeof(speed), &g_tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    gpio_set_level(STEP_MOTOR_GPIO_EN, 1); // disable motor

    // init current position
    g_curr_pos = MAX_STEP / 2;
    g_last_dir = STEP_MOTOR_SPIN_DIR_CLOCKWISE;
}

TaskHandle_t get_step_motor_task_handle(void)
{
    return g_step_motor_task_handle;
}
