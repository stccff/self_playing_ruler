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
#include "nvs_flash.h"
#include "nvs.h"
#include "play.h"
#include "digital_mic.h"
#include "gpio_pin_config.h"
#include "h_bridge.h"

/* ***************************************************************************************************************** */
/*                                                 macro define                                                            */
/* ***************************************************************************************************************** */

// hardware A v1.0
#ifdef CONFIG_HW_A_VER_1_0

#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0

#define SAMPLE_POINTS (20 * (1 << MODE))
#define SPEED_LOW_HZ (500 * (1 << MODE)) // < SPEED_HZ
#define SPEED_HZ (1200 * (1 << MODE))

// ruler features
#define RULER_LEN_MIN 16.52   // mm
#define RULER_LEN_MAX 65.52  // mm

#define RULER_LEN_MUSIC_MIN 22.0 // mm
#define RULER_LEN_MUSIC_MAX 65.0 // mm
#endif

// hardware A v1.1
#ifdef CONFIG_HW_A_VER_1_1

#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 1

#define SAMPLE_POINTS (20 * (1 << MODE))
#define SPEED_LOW_HZ (500 * (1 << MODE)) // < SPEED_HZ
#define SPEED_HZ (2000 * (1 << MODE))

// ruler features
#define RULER_LEN_MIN 14.52   // mm
#define RULER_LEN_MAX 64.67  // mm

#define RULER_LEN_MUSIC_MIN 22.0 // mm
#define RULER_LEN_MUSIC_MAX 64.5 // mm
#endif


// hardware B v1.0
#ifdef CONFIG_HW_B_VER_1_0

#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 1

#define SAMPLE_POINTS (20 * (1 << MODE))
#define SPEED_LOW_HZ (500 * (1 << MODE)) // < SPEED_HZ
#define SPEED_HZ (1200 * (1 << MODE))

// ruler features
#define RULER_LEN_MIN 15.05   // mm
#define RULER_LEN_MAX 64.05  // mm

#define RULER_LEN_MUSIC_MIN 24.0 // mm
#define RULER_LEN_MUSIC_MAX 64 // mm
#endif

#define MODE 1 // 0: full step, 1: half step, 2: 1/4 step, 3: 1/8 step, 4: 1/16 step, 5: 1/32 step

#define LEN_PER_FULL_STEP 0.15 // mm
#define MAX_FULL_STEP 320
#define LEN_PER_STEP (LEN_PER_FULL_STEP / (1 << MODE))
#define MAX_STEP (MAX_FULL_STEP * (1 << MODE))
#define SCREW_BACKLASH (2 * (1 << MODE)) // step

#define RULLER_FREQ_SAMPLE_NUM 60
#define RULLER_FREQ_SAMPLE_TOLERANCE 0.2

#define STEP_MOTOR_ENABLE_LEVEL 0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

// ruler features
/* formula f=k/(L^2)+b, L=(k/(f-b))^(1/2) */
#define SLOPE_K 172767.3698
#define INTERCEPT_B 20.1147953

#define STORAGE_NAMESPACE "ruler_player"
#define FREQ_TABLE_KEY "freq_table"
#define FREQ_TABLE_FFT_SIZE 4096
#define FREQ_TABLE_FFT_DELAY 1000 // ms


/* ***************************************************************************************************************** */
/*                                                 struct define                                                     */
/* ***************************************************************************************************************** */
struct PosFreqTlb {
    int pos;
    float freq;
};
/* ***************************************************************************************************************** */
/*                                                global variable                                                           */
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

struct PosFreqTlb *g_pf_table = NULL;
size_t g_pf_table_num = 0;
/* ***************************************************************************************************************** */
/*                                           function prototype                                                      */
/* ***************************************************************************************************************** */
static float get_freq_by_formula(float len)
{
    return (double)SLOPE_K / pow(len, 2) + INTERCEPT_B;
}

/**
 * @brief Get the stepper motor absolute step by frequency
 *
 * @param target_freq
 * @return double
 */
int convert_freq_to_pos(double target_freq)
{
    // int table_num = sizeof(g_pf_table) / sizeof(g_pf_table[0]);
    int table_num = g_pf_table_num;
    if (g_pf_table == NULL || table_num == 0) {
        ESP_LOGE(TAG, "g_pf_table=%p, table_num=%d", g_pf_table, table_num);
        return -1;
    }

    // the boundary check
    if (target_freq > g_pf_table[0].freq) {
        ESP_LOGE(TAG, "freq:%lf is outof table", target_freq);
        return -1;
    }
    if (target_freq < g_pf_table[table_num-1].freq) {
        ESP_LOGE(TAG, "freq:%lf is outof table", target_freq);
        return -1;
    }

    // linear interpolation
    for (int i = 0; i < table_num - 1; i++) {
        // check the range
        if (g_pf_table[i].freq >= target_freq && target_freq >= g_pf_table[i+1].freq) {
            float f1 = g_pf_table[i].freq;
            float p1 = g_pf_table[i].pos;
            float f2 = g_pf_table[i+1].freq;
            float p2 = g_pf_table[i+1].pos;

            // do linear interpolation
            float t = (target_freq - f1) / (f2 - f1);
            float interpolated_pos = p1 + t * (p2 - p1);
            return round(interpolated_pos);
        }
    }

    return -1.0f; // 理论上不会执行到这里
}

/**
 * @brief convert the length(mm) to absolute step
 *
 * @param len
 * @return int
 */
int convert_len_to_pos(double len)
{
    if (len < RULER_LEN_MIN || len > RULER_LEN_MAX) {
        ESP_LOGE(TAG, "Invalid length: %f, valid range [%f, %f] mm", len, RULER_LEN_MIN, RULER_LEN_MAX);
        return -1;
    }
    return (int)round((len - RULER_LEN_MIN) / (RULER_LEN_MAX - RULER_LEN_MIN) * MAX_STEP);
}

static float convert_pos_to_len(int pos)
{
    if (pos < 0 || pos > MAX_STEP) {
        ESP_LOGE(TAG, "Invalid pos: %d, valid range [%d, %d]", pos, 0, MAX_STEP);
        return -1;
    }

    return (float)RULER_LEN_MIN + (RULER_LEN_MAX - RULER_LEN_MIN) * pos / MAX_STEP;
}

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

int stepper_motor_action_by_len(bool is_sync, double len)
{
    int rc = ESP_OK;
    int pos = convert_len_to_pos(len);
    if (pos < 0) {
        ESP_LOGE(TAG, "convert_len_to_pos fail! len=%f", len);
        return ESP_ERR_INVALID_RESPONSE;
    }
    rc = stepper_motor_action_by_pos(is_sync, pos);
    return rc;
}

/**
 * @brief stepper motor act as the input freq
 *
 * @param freq
 * @return int
 */
int stepper_motor_action_by_freq(bool is_sync, double freq)
{
    int rc = ESP_OK;
    int pos = convert_freq_to_pos(freq);
    if (pos < 0) {
        ESP_LOGE(TAG, "convert_freq_to_pos fail! freq=%f", freq);
        return ESP_ERR_INVALID_RESPONSE;
    }
    rc = stepper_motor_action_by_pos(is_sync, pos);
    ESP_LOGI(TAG, "freq = %f, pos = %d, len = %f", freq, pos, convert_pos_to_len(pos));
    return rc;
}

static void action_init(void)
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


static int measure_frequency(float len, float *freq)
{
    int rc = ESP_OK;

    rc = play_single_note_by_len(len);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "measure play len:%f fail", len);
        return rc;
    }
    float target = get_freq_by_formula(len);
    rc = get_sound_frequency(target*(1-RULLER_FREQ_SAMPLE_TOLERANCE), target*(1+RULLER_FREQ_SAMPLE_TOLERANCE), freq, true);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "measure get sound frequency fail");
        return rc;
    }

    vTaskDelay(pdMS_TO_TICKS(FREQ_TABLE_FFT_DELAY));

    return rc;
}

static int create_freq_table(void)
{
    int rc = ESP_OK;

    rc = sound_fft_init(FREQ_TABLE_FFT_SIZE);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sound FFT with error: %d", rc);
        g_pf_table = NULL;
        g_pf_table_num = 0;
        return rc;
    }

    // int step_mini_num = ((int)RULLER_FREQ_STEP_CHANGE_LEN - (int)RULER_LEN_MUSIC_MIN) * 2;
    // int step_normal_num = (int)RULER_LEN_MUSIC_MAX - (int)RULLER_FREQ_STEP_CHANGE_LEN + 1;
    // g_pf_table_num = step_mini_num + step_normal_num;
    g_pf_table_num = RULLER_FREQ_SAMPLE_NUM;

    if (g_pf_table != NULL) {
        free(g_pf_table);
        g_pf_table = NULL;
    }
    g_pf_table = (struct PosFreqTlb *)malloc(sizeof(struct PosFreqTlb) * g_pf_table_num);
    if (g_pf_table == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for frequency table");
        g_pf_table_num = 0;
        return ESP_ERR_NO_MEM;
    }

    // remove backlash error
    rc = play_single_note_by_pos(0);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "remove backlash, set pos error");
        return rc;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    double step_len_start = LEN_PER_FULL_STEP;
    double Sn_1 = RULER_LEN_MUSIC_MAX - RULER_LEN_MUSIC_MIN;
    size_t n = g_pf_table_num;
    float a1 = step_len_start;
    double d = (Sn_1 - (n-1) * a1) * 2 / ((n-1) * (n-2));
    float x1 = RULER_LEN_MUSIC_MIN;
    for (int i = 0; i < g_pf_table_num; i++) { // i --> n-1
        double len = x1 + i * a1 + i * (i - 1) * d / 2;
        int pos = convert_len_to_pos(len);
        if (pos < 0) {
            ESP_LOGE(TAG, "convert_len_to_pos fail! length: %f", len);
            free(g_pf_table);
            g_pf_table = NULL;
            g_pf_table_num = 0;
            return ESP_FAIL;
        }
        g_pf_table[i].pos = pos;
        rc = measure_frequency(len, &g_pf_table[i].freq);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Failed to measure frequency for length: %f", len);
            free(g_pf_table);
            g_pf_table = NULL;
            g_pf_table_num = 0;
            return rc;
        }
    }

    // print

    return ESP_OK;
}

int freq_table_init(bool force_init) // TODO: need check stepper motor controler's mode, ruler len, before use nvs F-P table
{
    int rc = ESP_OK;

    // Open
    nvs_handle_t nvs_handle;
    rc = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(rc));
        return rc;
    }

    if (force_init) {
        /* create table and write to nvs */
        // creat table
        rc = create_freq_table();
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Error (%s) creating frequency table", esp_err_to_name(rc));
            goto err;
        }
        // Write blob
        rc = nvs_set_blob(nvs_handle, FREQ_TABLE_KEY, g_pf_table, g_pf_table_num * sizeof(struct PosFreqTlb));
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Error (%s) writing NVS blob", esp_err_to_name(rc));
            goto err;
        }
        // Commit
        rc = nvs_commit(nvs_handle);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Error (%s) committing NVS changes", esp_err_to_name(rc));
            goto err;
        }
        ESP_LOGI(TAG, "Frequency table created and written to NVS, index num: %d", g_pf_table_num);
    } else {
        // check if the frequency table is exists
        size_t tlb_size = 0;
        rc = nvs_get_blob(nvs_handle, FREQ_TABLE_KEY, NULL, &tlb_size);
        if (rc != ESP_OK) {
            if (rc == ESP_ERR_NVS_NOT_FOUND) {
                /* nvs not exist */
                rc = ESP_OK;
                ESP_LOGW(TAG, "Frequency table not found in NVS");
            } else {
                // Other error
                ESP_LOGE(TAG, "Error (%s) reading NVS blob size", esp_err_to_name(rc));
                goto err;
            }
        } else {
            /* nvs exist: get table from nvs */
            if (g_pf_table != NULL) {
                free(g_pf_table);
                g_pf_table = NULL;
                g_pf_table_num = 0;
            }
            g_pf_table = (struct PosFreqTlb *)malloc(tlb_size);
            if (g_pf_table == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for getting frequency table");
                g_pf_table_num = 0;
                rc = ESP_ERR_NO_MEM;
                goto err;
            }
            int err = nvs_get_blob(nvs_handle, FREQ_TABLE_KEY, g_pf_table, &tlb_size);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error (%s) reading NVS blob", esp_err_to_name(err));
                goto err;
            }
            g_pf_table_num = tlb_size / sizeof(struct PosFreqTlb);
            ESP_LOGI(TAG, "Frequency table loaded from NVS, index num: %d", g_pf_table_num);
        }
    }

err:
    if (rc != ESP_OK) {
        if (g_pf_table != NULL) {
            free(g_pf_table);
            g_pf_table = NULL;
        }
        g_pf_table_num = 0;
    }
    nvs_close(nvs_handle);
    return rc;

    // check tbale valiation(1.is sorted, 2.check freq is accurate)
}

int freq_table_clear(void)
{
    int rc = ESP_OK;
    // Open
    nvs_handle_t nvs_handle;
    rc = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(rc));
        return rc;
    }
    // Write blob
    rc = nvs_erase_key(nvs_handle, FREQ_TABLE_KEY);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) erase key", esp_err_to_name(rc));
        goto err;
    }
    // Commit
    rc = nvs_commit(nvs_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) committing NVS changes", esp_err_to_name(rc));
        goto err;
    }
err:
    nvs_close(nvs_handle);

    return rc;
}

int freq_table_show(void)
{
    int rc = ESP_OK;
    if (g_pf_table == NULL) {
        ESP_LOGW(TAG, "frequency table is NULL");
    } else {
        ESP_LOGI(TAG, "frequency table:");
        printf("Estimated_length\tPosition\tFrequency\n");
        for (size_t i = 0; i < g_pf_table_num; i++) {
            printf("%f\t%d\t%f\n", convert_pos_to_len(g_pf_table[i].pos), g_pf_table[i].pos, g_pf_table[i].freq);
        }
    }
    return rc;
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

void stepper_motor_init(void)
{
    /* stepper motor action init */
    action_init();
    /* init frequncy table */
    ESP_ERROR_CHECK(freq_table_init(false));
}
