#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "servo_motor.h"
#include "step_motor.h"
#include "ruler.h"
#include "h_bridge.h"
#include "note_decode.h"
#include "driver/gptimer.h"
#include "play.h"


/* ***************************************************************************************************************** */
/*                                               macro define                                                        */
/* ***************************************************************************************************************** */
#define TAG "play"

#define POLARITY_POSITIVE 1
#define POLARITY_NEGATIVE -1

#define E_MAGNET_FINAL_RELEASE_DELAY 10*1000 // ms
/* ***************************************************************************************************************** */
/*                                               struct define                                                       */
/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */
/*                                                type define                                                        */
/* ***************************************************************************************************************** */
typedef enum {
    ACT_POS,
    ACT_LEN,
    ACT_FREQ,
    ACT_MAX_NUM,
} stepper_motor_act_t;
/* ***************************************************************************************************************** */
/*                                               global variable                                                     */
/* ***************************************************************************************************************** */
gptimer_handle_t g_strum_timer = NULL;
gptimer_handle_t g_emagnet_off_timer = NULL;
bool g_is_emagnet_off_timer_started = false;
int64_t alarm_time = 0; // for debug
SemaphoreHandle_t g_mutex = NULL;
/* ***************************************************************************************************************** */
/*                                              function prototype                                                   */
/* ***************************************************************************************************************** */
static bool IRAM_ATTR timer_strum_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    // stop timer immediately
    gptimer_stop(timer);
    alarm_time = esp_timer_get_time();
    // strum
    servo_strum_iram_without_fpu();

    // xQueueSendFromISR(queue, &ele, &high_task_awoken);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

static bool IRAM_ATTR timer_idel_release_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    // stop timer immediately
    gptimer_stop(timer);
    // release the e-magnet
    h_bridge_set(0, 0);
    h_bridge_set(1, 0);
    g_is_emagnet_off_timer_started = false;

    return (high_task_awoken == pdTRUE);
}

void play_init(void)
{
    /* for protect */
    g_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(g_mutex == NULL);

    /* timer for strum */
    ESP_LOGI(TAG, "Create stepper motor timer handle");
    gptimer_config_t strum_timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&strum_timer_config, &g_strum_timer));

    gptimer_event_callbacks_t strum_cbs = {
        .on_alarm = timer_strum_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_strum_timer, &strum_cbs, NULL));

    ESP_LOGI(TAG, "Enable strum timer");
    ESP_ERROR_CHECK(gptimer_enable(g_strum_timer));


    /* timer for release press, when there is no note to play */
    ESP_LOGI(TAG, "Create e-magnet release timer handle");
    gptimer_config_t off_timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&off_timer_config, &g_emagnet_off_timer));

    gptimer_event_callbacks_t off_strum_cbs = {
        .on_alarm = timer_idel_release_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_emagnet_off_timer, &off_strum_cbs, NULL));

    ESP_LOGI(TAG, "Enable emagnet auto off timer");
    ESP_ERROR_CHECK(gptimer_enable(g_emagnet_off_timer));
}


static void start_emagnet_off_timer(int delay_ms)
{

    ESP_ERROR_CHECK(gptimer_set_raw_count(g_emagnet_off_timer, 0)); // reset counter
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = delay_ms * 1000, // convert to us
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(g_emagnet_off_timer, &alarm_config));
    if (g_is_emagnet_off_timer_started == false) {
        ESP_ERROR_CHECK(gptimer_start(g_emagnet_off_timer));
    }
    g_is_emagnet_off_timer_started = true;
}

int play_single_note_by_pos(int pos)
{
    int rc = ESP_OK;

    rc = xSemaphoreTake(g_mutex, 0);
    if (rc != pdTRUE) {
        ESP_LOGE(TAG, "play function should not be called reentrantly");
        return ESP_ERR_INVALID_STATE;
    }

    /* estimate stepper motor action time */
    int ruler_estimated_time = calc_stepper_motor_time_by_pos(pos);
    if (ruler_estimated_time < 0) {
        rc = ruler_estimated_time;
        goto err;
    }
    ESP_LOGD(TAG, "stepper motor action estimated time: %d ms", ruler_estimated_time);

    /* release the ruler */
    if (ruler_estimated_time != 0) { // if zero, no need to release
        h_bridge_set(0, 0);
        h_bridge_set(1, 0);
    }
    start_emagnet_off_timer(E_MAGNET_FINAL_RELEASE_DELAY);
    vTaskDelay(pdMS_TO_TICKS(20)); // Waiting for demagnetization, 20 is ok? or 30?

    /* move stepper motor and strum */
    if (ruler_estimated_time <= SERVO_STRUM_PREPARE_TIME) { // stepper motor cost < servo prepare in 6v, immidiately strum
        servo_motor_action(3); // strum start

        rc = stepper_motor_action_by_pos(true, pos);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "play fail, pos: %d", pos);
            goto err;
        }

        // after ruler move, press the ruler
        h_bridge_set(0, POLARITY_POSITIVE);
        h_bridge_set(1, POLARITY_NEGATIVE);

        // strum: now touch the ruler(triggered by action(3))

        vTaskDelay(pdMS_TO_TICKS(SERVO_STRUM_START_2_RELEASE_TIME - ruler_estimated_time));
        // strum end
    } else {
        // start timer (time alart time: ruler_estimated_time - SERVO_STRUM_PREPARE_TIME)
        int64_t start_time = esp_timer_get_time();
        ESP_LOGD(TAG, "timer start time in: %llu ms, set timer: %d ms", start_time / 1000, ruler_estimated_time - SERVO_STRUM_PREPARE_TIME);

        ESP_ERROR_CHECK(gptimer_set_raw_count(g_strum_timer, 0)); // reset counter
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = (ruler_estimated_time - SERVO_STRUM_PREPARE_TIME) * 1000,
        };
        ESP_ERROR_CHECK(gptimer_set_alarm_action(g_strum_timer, &alarm_config));
        ESP_ERROR_CHECK(gptimer_start(g_strum_timer)); // strum in timer callback

        rc = stepper_motor_action_by_pos(true, pos);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "play fail, pos: %d", pos);
            goto err;
        }

        // after ruler move, press the ruler
        h_bridge_set(0, POLARITY_POSITIVE);
        h_bridge_set(1, POLARITY_NEGATIVE);

        vTaskDelay(pdMS_TO_TICKS(SERVO_STRUM_START_2_RELEASE_TIME - SERVO_STRUM_PREPARE_TIME));

        ESP_LOGD(TAG, "timer alarm time in: %llu ms, timer cost: %llu ms", alarm_time / 1000, (alarm_time - start_time) / 1000);
    }

err:
    ESP_ERROR_CHECK(xSemaphoreGive(g_mutex) == pdFALSE);
    return rc;
}

/**
 * @brief Play a note by frequency, blocking function
 *
 * @param freq
 * @return int
 */
int play_single_note_by_freq(float freq)
{
    int pos = convert_freq_to_pos(freq);
    return play_single_note_by_pos(pos);
}

int play_single_note_by_len(float len)
{
    int pos = convert_len_to_pos(len);
    return play_single_note_by_pos(pos);
}

int play_single_note_by_midi(int midi)
{
    if (midi < 0 || midi > 127) {
        ESP_LOGE(TAG, "Invalid MIDI number: %d", midi);
        return ESP_ERR_INVALID_ARG;
    }
    float freq = convert_midi_to_freq(midi);
    return play_single_note_by_freq(freq);
}
