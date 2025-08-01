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
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "step_motor.h"
#include "play.h"
#include "digital_mic.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "ruler.h"

/* ***************************************************************************************************************** */
/*                                                 macro define                                                      */
/* ***************************************************************************************************************** */
// hardware A >=v1.1
#ifdef CONFIG_HW_A_VER_1_1
#define RULER_LEN_MIN 15.67   // mm
#define RULER_LEN_MAX 64.67  // mm

#define RULER_LEN_MUSIC_MIN 23.0 // mm
#define RULER_LEN_MUSIC_MAX 64.5 // mm
#endif

#define RULLER_FREQ_SAMPLE_NUM 60
#define RULLER_FREQ_SAMPLE_TOLERANCE 0.25

/* formula f=k/(L^2)+b, L=(k/(f-b))^(1/2) */
#define SLOPE_K 217560.0345
#define INTERCEPT_B 10.14935316

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
/*                                                global variable                                                    */
/* ***************************************************************************************************************** */
static const char *TAG = "ruler";
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

int ruler_action_by_len(bool is_sync, double len)
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
int ruler_action_by_freq(bool is_sync, double freq)
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
        ESP_LOGE(TAG, "measure get sound frequency fail, rc = %d", rc);
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

    // enable mic
    mic_reconfig_sample_rate(8000);
    mic_enable(true);


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

    double step_len_start = FULL_STEP_LEN * 2;
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
            rc = ESP_FAIL;
            goto err;
        }
        g_pf_table[i].pos = pos;
        rc = measure_frequency(len, &g_pf_table[i].freq);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Failed to measure frequency for length: %f", len);
            free(g_pf_table);
            g_pf_table = NULL;
            g_pf_table_num = 0;
            goto err;
        }
    }

err:
    mic_enable(false); // disable mic channel

    return ESP_OK;
}
/**
 * @brief Initialize the frequency table
 *
 * @param force_init if true, it will create a new table and write to nvs.
 *                   If false, it will read out the exists from nvs. if not exist, it will create a new table.
 * @return int
 */
int freq_table_init(bool force_init) // TODO: Need to check if the f-p table is legacy before using it.
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

void ruler_init(void)
{
    /* init frequncy table */
    ESP_ERROR_CHECK(freq_table_init(false));
}
