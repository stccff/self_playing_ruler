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
#include "nls.h"
#include "note_decode.h"
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
#define FREQ_TABLE_FFT_SIZE 2048
#define MEASURE_FFT_DELAY 100 // ms

/* ***************************************************************************************************************** */
/*                                                 struct define                                                     */
/* ***************************************************************************************************************** */
struct freq_table {
    int pos;
    float freq;
};
struct PosFreqTlb {
    struct freq_table table[RULLER_FREQ_SAMPLE_NUM];
    double k;
    double a;
    double b;
};
/* ***************************************************************************************************************** */
/*                                                global variable                                                    */
/* ***************************************************************************************************************** */
static const char *TAG = "RULER";
// struct PosFreqTlb *g_pf_table = NULL;
// size_t g_pf_table_num = 0;
static struct PosFreqTlb *g_pf_table = NULL;
bool g_is_formula = false; // use formula to calculate pos by freq, or use table
/* ***************************************************************************************************************** */
/*                                           function prototype                                                      */
/* ***************************************************************************************************************** */

/* formula f=k/(L^2)+b, L=(k/(f-b))^(1/2) */
static float calculate_freq_by_len(float len)
{
    return (double)SLOPE_K / pow(len, 2) + INTERCEPT_B;
}

/* formula f=k/(pos + a)^2 + b, pos = sqrt(k/(f-b)) - a */
static int calculate_pos_by_freq(float freq)
{
    double k = g_pf_table->k;
    double a = g_pf_table->a;
    double b = g_pf_table->b;
    return round(sqrt(k / (freq - b)) - a);
}

static float calculate_freq_by_pos(int pos)
{
    double k = g_pf_table->k;
    double a = g_pf_table->a;
    double b = g_pf_table->b;
    return k / pow(pos + a, 2) + b;
}

/**
 * @brief Get the stepper motor absolute step by frequency
 *
 * @param target_freq
 * @return double
 */
int convert_freq_to_pos(double target_freq)
{
    if (g_pf_table == NULL) {
        ESP_LOGE(TAG, "frequency not be initialized!");
        return -1;
    }

    struct freq_table *table = g_pf_table->table;
    int num = RULLER_FREQ_SAMPLE_NUM;

    // the boundary check
    if (target_freq > table[0].freq) {
        ESP_LOGE(TAG, "freq:%lf is outof table", target_freq);
        return -1;
    }
    if (target_freq < table[num-1].freq) {
        ESP_LOGE(TAG, "freq:%lf is outof table", target_freq);
        return -1;
    }

    if (g_is_formula) {
        return calculate_pos_by_freq(target_freq);
    } else {
        // linear interpolation
        for (int i = 0; i < num - 1; i++) {
            // check the range
            if (table[i].freq >= target_freq && target_freq >= table[i+1].freq) {
                float f1 = table[i].freq;
                float p1 = table[i].pos;
                float f2 = table[i+1].freq;
                float p2 = table[i+1].pos;

                // do linear interpolation
                float t = (target_freq - f1) / (f2 - f1);
                float interpolated_pos = p1 + t * (p2 - p1);
                return (int)round(interpolated_pos);
            }
        }
    }

    return -1.0f; // 理论上不会执行到这里
}


/**
 * @brief convert the length(mm) to absolute step. (not accurate)
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

// int ruler_action_by_len(bool is_sync, double len)
// {
//     int rc = ESP_OK;
//     int pos = convert_len_to_pos(len);
//     if (pos < 0) {
//         ESP_LOGE(TAG, "convert_len_to_pos fail! len=%f", len);
//         return ESP_ERR_INVALID_RESPONSE;
//     }
//     rc = stepper_motor_action_by_pos(is_sync, pos);
//     return rc;
// }

// /**
//  * @brief stepper motor act as the input freq
//  *
//  * @param freq
//  * @return int
//  */
// int ruler_action_by_freq(bool is_sync, double freq)
// {
//     int rc = ESP_OK;
//     int pos = convert_freq_to_pos(freq);
//     if (pos < 0) {
//         ESP_LOGE(TAG, "convert_freq_to_pos fail! freq=%f", freq);
//         return ESP_ERR_INVALID_RESPONSE;
//     }
//     rc = stepper_motor_action_by_pos(is_sync, pos);
//     ESP_LOGI(TAG, "freq = %f, pos = %d, len = %f", freq, pos, convert_pos_to_len(pos));
//     return rc;
// }

static int measure_frequency_by_len(float len, float *freq)
{
    int rc = ESP_OK;

    rc = play_single_note_by_len(len);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "measure play len:%f fail", len);
        return rc;
    }
    float target = calculate_freq_by_len(len);
    rc = get_sound_frequency(target*(1-RULLER_FREQ_SAMPLE_TOLERANCE), target*(1+RULLER_FREQ_SAMPLE_TOLERANCE), freq, true);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "measure get sound frequency fail, rc = %d", rc);
        return rc;
    }

    vTaskDelay(pdMS_TO_TICKS(MEASURE_FFT_DELAY));

    return rc;
}



static void calculate_parameters(void)
{
    double x[RULLER_FREQ_SAMPLE_NUM];
    double y[RULLER_FREQ_SAMPLE_NUM];
    for (int i = 0; i < RULLER_FREQ_SAMPLE_NUM; i++) {
        x[i] = g_pf_table->table[i].pos;
        y[i] = g_pf_table->table[i].freq;
    }

    // 初始参数估计 (k, a, b)
    double p0[3] = {1.0, 0.0, 0.0};

    // 执行拟合
    lm_fit(RULLER_FREQ_SAMPLE_NUM, x, y, p0, 100, 1e-6);

    g_pf_table->k = p0[0];
    g_pf_table->a = p0[1];
    g_pf_table->b = p0[2];

    ESP_LOGI(TAG, "fiting result: k=%f, a=%f, b=%f", p0[0], p0[1], p0[2]);
}

static int create_freq_table(void)
{
    int rc = ESP_OK;

    rc = sound_fft_init(FREQ_TABLE_FFT_SIZE);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "create frequency table fail with fft init error: %d", rc);
        return rc;
    }

    // enable mic
    mic_reconfig_sample_rate(8000);
    mic_enable(true);

    if (g_pf_table != NULL) {
        free(g_pf_table);
        g_pf_table = NULL;
    }
    g_pf_table = (struct PosFreqTlb *)malloc(sizeof(struct PosFreqTlb));
    if (g_pf_table == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for frequency table");
        return ESP_ERR_NO_MEM;
    }
    struct freq_table *table = g_pf_table->table;

    // remove backlash error
    rc = stepper_motor_action_by_pos(true, 0);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "remove backlash, set pos error");
        goto err;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    /* make step len by using arithmetic progression */
    double step_len_start = FULL_STEP_LEN * 2;
    double Sn_1 = RULER_LEN_MUSIC_MAX - RULER_LEN_MUSIC_MIN;
    size_t n = RULLER_FREQ_SAMPLE_NUM;
    float a1 = step_len_start;
    double d = (Sn_1 - (n-1) * a1) * 2 / ((n-1) * (n-2));
    float x1 = RULER_LEN_MUSIC_MIN;
    for (int i = 0; i < RULLER_FREQ_SAMPLE_NUM; i++) { // i --> n-1
        double len = x1 + i * a1 + i * (i - 1) * d / 2;
        int pos = convert_len_to_pos(len);
        if (pos < 0) {
            ESP_LOGE(TAG, "convert_len_to_pos fail! length: %f", len);
            rc = ESP_FAIL;
            goto err;
        }
        table[i].pos = pos;
        rc = measure_frequency_by_len(len, &table[i].freq);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Failed to measure frequency for length: %f", len);
            goto err;
        }
    }

    calculate_parameters();

err:
    if (rc != ESP_OK) {
        free(g_pf_table);
        g_pf_table = NULL;
    }

    mic_enable(false); // disable mic channel

    return rc;
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
        rc = nvs_set_blob(nvs_handle, FREQ_TABLE_KEY, g_pf_table, sizeof(struct PosFreqTlb));
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
        ESP_LOGI(TAG, "Frequency table created and written to NVS");
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
            }
            if (tlb_size != sizeof(struct PosFreqTlb)) {
                ESP_LOGE(TAG, "Frequency table size mismatch, expected: %zu, got: %zu", sizeof(struct PosFreqTlb), tlb_size);
                rc = ESP_ERR_INVALID_SIZE;
                goto err;
            }
            g_pf_table = (struct PosFreqTlb *)malloc(sizeof(struct PosFreqTlb));
            if (g_pf_table == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for getting frequency table");
                rc = ESP_ERR_NO_MEM;
                goto err;
            }
            int err = nvs_get_blob(nvs_handle, FREQ_TABLE_KEY, g_pf_table, &tlb_size);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error (%s) reading NVS blob", esp_err_to_name(err));
                goto err;
            }
            ESP_LOGI(TAG, "Frequency table loaded from NVS");
        }
    }

err:
    if (rc != ESP_OK) {
        if (g_pf_table != NULL) {
            free(g_pf_table);
            g_pf_table = NULL;
        }
    }
    nvs_close(nvs_handle);
    return rc;
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
        struct freq_table *table = g_pf_table->table;
        for (size_t i = 0; i < RULLER_FREQ_SAMPLE_NUM; i++) {
            printf("%f\t%d\t%f\n", convert_pos_to_len(table[i].pos), table[i].pos, table[i].freq);
        }
    }
    return rc;
}

void freq_table_use_formula(bool is_formula)
{
    g_is_formula = is_formula;
    ESP_LOGI(TAG, "Set frequency table formula mode: %s", is_formula ? "formula" : "table");
}

int recalculate_params(void)
{
    int rc = ESP_OK;

    if (g_pf_table == NULL) {
        ESP_LOGE(TAG, "Frequency table is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Open
    nvs_handle_t nvs_handle;
    rc = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(rc));
        return rc;
    }

    // Recalculate parameters
    calculate_parameters();

    // Write blob
    rc = nvs_set_blob(nvs_handle, FREQ_TABLE_KEY, g_pf_table, sizeof(struct PosFreqTlb));
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
err:
    nvs_close(nvs_handle);

    return rc;
}


// static int test_midi_freq() // TODO:

int pitch_accuracy_test(void)
{
    int rc = ESP_OK;

    if (g_pf_table == NULL) {
        ESP_LOGE(TAG, "Frequency table is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    rc = sound_fft_init(FREQ_TABLE_FFT_SIZE);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "create frequency table fail with fft init error: %d", rc);
        return rc;
    }

    // enable mic
    mic_reconfig_sample_rate(8000);
    mic_enable(true);

    // get midi range
    double max_freq = calculate_freq_by_pos(g_pf_table->table[0].pos);
    double min_freq = calculate_freq_by_pos(g_pf_table->table[RULLER_FREQ_SAMPLE_NUM - 1].pos);
    int midi_max = find_midi_idx_by_freq(max_freq, false);
    int midi_min = find_midi_idx_by_freq(min_freq, true);

    if (midi_min < 0 || midi_max < 0) {
        ESP_LOGE(TAG, "Failed to find MIDI range for frequency: min=%f, max=%f", min_freq, max_freq);
        goto err;

    }

    printf("midi_num target_freq measured_freq error\n");
    for (int i = midi_min; i <= midi_max; i++) {
    // for (int i = midi_max; i >= midi_min; i--) {
        rc = play_single_note_by_midi(i);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Failed to play MIDI note: %d", i);
            continue;
        }

        float target_freq = convert_midi_to_freq(i);
        float measured_freq;
        rc = get_sound_frequency(target_freq * (1 - RULLER_FREQ_SAMPLE_TOLERANCE), target_freq * (1 + RULLER_FREQ_SAMPLE_TOLERANCE),
                                    &measured_freq, false);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get sound frequency for MIDI %d", i);
            continue;
        }

        printf("%d %f %f %f%%\n", i, target_freq, measured_freq, (measured_freq - target_freq) / target_freq * 100);

        vTaskDelay(pdMS_TO_TICKS(MEASURE_FFT_DELAY));
    }

err:
    mic_enable(false); // disable mic channel

    return rc;

}


void ruler_init(void)
{
    /* init frequncy table */
    int rc = freq_table_init(false);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize frequency table with error: %d", rc);
        return;
    }
}
