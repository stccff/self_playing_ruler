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

/* ***************************************************************************************************************** */
/*                                                 macro define                                                            */
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

#define MODE 1 // 0: full step, 1: half step, 2: 1/4 step, 3: 1/8 step, 4: 1/16 step, 5: 1/32 step

#define SAMPLE_POINTS (20 * (1 << MODE))
#define SPEED_LOW_HZ (500 * (1 << MODE)) // < SPEED_HZ
#define SPEED_HZ (1200 * (1 << MODE))

#define LEN_PER_FULL_STEP 0.15 // mm
#define MAX_FULL_STEP 300 // TODO: test
#define LEN_PER_STEP (LEN_PER_FULL_STEP / (1 << MODE))
#define MAX_STEP (MAX_FULL_STEP * (1 << MODE))
#define LEAD_SCREW_BACKLASH (2 * (1 << MODE)) // step

// ruler features
#define RULER_LEN_MIN 13.3   // mm
#define RULER_LEN_MAX 65.52  // mm

#define RULER_LEN_MUSIC_MIN 22.0 // mm
#define RULER_LEN_MUSIC_MAX 62.0 // mm

#define RULLER_FREQ_STEP_CHANGE_LEN 3.0 // mm

// #define RULER_FREQ_MIN 73.42    // ≈55.80mm
// #define RULER_FREQ_MAX 349.23   // ≈25.61mm

/* formula f=k/(L^2), L=(k/f)^(1/2) */
#define k 194543.9295 // not ok


#define STORAGE_NAMESPACE "ruler_player"
#define FREQ_TABLE_KEY "freq_table"


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
size_t g_pf_table_size = 0;
/* ***************************************************************************************************************** */
/*                                           function prototype                                                      */
/* ***************************************************************************************************************** */
/**
 * @brief Get the stepper motor absolute step by frequency
 * 
 * @param target_freq 
 * @return double 
 */
static double get_pos_by_freq(double target_freq) {
    // int table_size = sizeof(g_pf_table) / sizeof(g_pf_table[0]);
    int table_size = g_pf_table_size;
    if (g_pf_table == NULL || table_size == 0) {
        ESP_LOGE(TAG, "g_pf_table=%p, table_size=%d", g_pf_table, table_size);
        return -1;
    }

    // the boundary check
    if (target_freq >= g_pf_table[0].freq) {
        ESP_LOGE(TAG, "freq:%lf in outof table", target_freq);
        return -1;
    }
    if (target_freq <= g_pf_table[table_size-1].freq) {
        ESP_LOGE(TAG, "freq:%lf in outof table", target_freq,);
        return -1;
    }

    // linear interpolation
    for (int i = 0; i < table_size - 1; i++) {
        // check the range
        if (g_pf_table[i].freq >= target_freq && target_freq >= g_pf_table[i+1].freq) {
            float f1 = g_pf_table[i].freq;
            float l1 = g_pf_table[i].len;
            float f2 = g_pf_table[i+1].freq;
            float l2 = g_pf_table[i+1].len;

            // do linear interpolation
            float t = (target_freq - f1) / (f2 - f1);
            float interpolated_len = l1 + t * (l2 - l1);
            return interpolated_len;
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
static int convert_len_to_pos(double len)
{
    if (len < RULER_LEN_MIN || len > RULER_LEN_MAX) {
        ESP_LOGE(TAG, "Invalid length: %f", len);
        return -1;
    }
    return (int)((len - RULER_LEN_MIN) / (RULER_LEN_MAX - RULER_LEN_MIN) * MAX_STEP);
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

int step_motor_action_by_len(double len)
{
    int pos = convert_len_to_pos(len);
    if (pos < 0) {
        ESP_LOGE(TAG, "convert_len_to_pos fail! len=%f", len);
        return 0;
    }
    return stepper_motor_action_by_pos(pos);
}

int stepper_motor_action_by_pos(int pos)
{
    int step = convert_pos_to_step(pos);
    if (step == 0) {
        return step;
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
    // wait all transactions finished
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(pdMS_TO_TICKS(10)); //TODO: is need?
    gpio_set_level(STEP_MOTOR_GPIO_EN, 1); // disable motor

    return step;
}

/**
 * @brief stepper motor act as the input freq
 * 
 * @param freq 
 * @return int step numbers 
 */
int stepper_motor_action(double freq)
{
    int pos = get_pos_by_freq(freq);
    if (pos < 0) {
        ESP_LOGE(TAG, "get_pos_by_freq fail! freq=%f", freq);
        return 0;
    }
    int step = stepper_motor_action_by_pos(pos);
    // ESP_LOGI(TAG, "freq = %f, len = %f, step = %d", freq, len, step); // TODO:
    return step;
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

    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    g_tx_config.loop_count = MAX_STEP + 10 * (1 << MODE);
    uint32_t speed = SPEED_LOW_HZ;
    ESP_ERROR_CHECK(rmt_transmit(g_motor_chan, g_uniform_motor_encoder, &speed, sizeof(speed), &g_tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_motor_chan, -1));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
    g_tx_config.loop_count = MAX_STEP + 10 * (1 << MODE);
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

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}


static int creat_freq_table(void)
{
    int rc = ESP_OK;

    int step_mini_num = ((int)RULLER_FREQ_STEP_CHANGE_LEN - (int)RULER_LEN_MUSIC_MIN) * 2 + 1;
    int step_normal_num = (int)RULER_LEN_MUSIC_MAX - (int)RULLER_FREQ_STEP_CHANGE_LEN;
    if (g_pf_table != NULL) {
        free(g_pf_table);
        g_pf_table = NULL;
        g_pf_table_size = 0;
    }

    g_pf_table_size = step_mini_num + step_normal_num;
    g_pf_table = (struct PosFreqTlb *)malloc(sizeof(struct PosFreqTlb) * g_pf_table_size);
    if (g_pf_table == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for frequency table");
        g_pf_table_size = 0;
        return ESP_ERR_NO_MEM;
    }

    float step_len = 0;
    for (int i = 0; i < g_pf_table_size; i++) {
        if (i < step_mini_num) {
            float step_len = 0.5;
        } else {
            step_len = 1;
        }
        float len = RULER_LEN_MUSIC_MIN + i * step_len;
        g_pf_table[i].pos = convert_len_to_pos(len);
        rc = measure_frequency(len, &g_pf_table[i].freq); // TODO: implement measure_frequency
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Failed to measure frequency for length: %f", len);
            free(g_pf_table);
            g_pf_table = NULL;
            g_pf_table_size = 0;
            return rc;
        }
    }

    // print

    return ESP_OK;
}

static int freq_table_init(bool force_init)
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
        rc = creat_freq_table();
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Error (%s) creating frequency table", esp_err_to_name(rc));
            goto err;
        }
        // Write blob
        rc = nvs_set_blob(nvs_handle, FREQ_TABLE_KEY, g_pf_table, g_pf_table_size);
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
        ESP_LOGI(TAG, "Frequency table created and written to NVS, index num: %d", g_pf_table_size);
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
                g_pf_table_size = 0;
            }
            g_pf_table = (struct PosFreqTlb *)malloc(tlb_size);
            if (g_pf_table == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for getting frequency table");
                g_pf_table_size = 0;
                rc = ESP_ERR_NO_MEM;
                goto err;
            }
            int err = nvs_get_blob(nvs_handle, FREQ_TABLE_KEY, g_pf_table, tlb_size);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error (%s) reading NVS blob", esp_err_to_name(err));
                goto err;
            }
            g_pf_table_size = tlb_size;
            ESP_LOGI(TAG, "Frequency table loaded from NVS, index num: %d", tlb_size / sizeof(struct PosFreqTlb));
        }        
    }

err:
    if (rc != ESP_OK) {
        if (g_pf_table != NULL) {
            free(g_pf_table);
            g_pf_table = NULL;
        }
        g_pf_table_size = 0;
    }
    nvs_close(nvs_handle);
    return rc;

    // check tbale valiation(1.is sorted, 2.check freq is accurate)
}

void stepper_motor_init(void)
{
    /* stepper motor action init */
    action_init();
    /* init nvs */
    init_nvs();
    /* init frequncy table */
    ESP_ERROR_CHECK(freq_table_init(false)); // TODO:
}


