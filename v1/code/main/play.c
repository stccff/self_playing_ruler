#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "play.h"
#include "servo_motor.h"
#include "step_motor.h"
#include "electromagnet.h"
#include "note_decode.h"
#include "driver/gptimer.h"

/* ***************************************************************************************************************** */
/*                                               macro define                                                        */
/* ***************************************************************************************************************** */
#define TAG "play"

#define POLARITY_POSITIVE 1
#define POLARITY_NEGATIVE -1
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
gptimer_handle_t g_gptimer = NULL;
#ifdef CONFIG_DEBUG_PRINT
int64_t alarm_time = 0;
#endif
/* ***************************************************************************************************************** */
/*                                              function prototype                                                   */
/* ***************************************************************************************************************** */
static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    // stop timer immediately
    gptimer_stop(timer);
#ifdef CONFIG_DEBUG_PRINT
    alarm_time = esp_timer_get_time();
#endif
    // strum
    servo_motor_action(3);

    // xQueueSendFromISR(queue, &ele, &high_task_awoken);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

void play_timer_init(void)
{
    // QueueHandle_t queue = xQueueCreate(10, sizeof(example_queue_element_t));
    // if (!queue) {
    //     ESP_LOGE(TAG, "Creating queue failed");
    //     return;
    // }

    ESP_LOGI(TAG, "Create stepper motor timer handle");
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &g_gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_gptimer, &cbs, NULL));

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(g_gptimer));

    // ESP_LOGI(TAG, "Start timer, stop it at alarm event");

    ESP_LOGI(TAG, "Create e-magnet timer handle");
}

static int play_single_note(stepper_motor_act_t act_type, void *param)
{
    int rc = ESP_OK;
    /* release the ruler */
    // electromagnet_set(0, POLARITY_NEGATIVE);
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    electromagnet_set(0, 0);
    electromagnet_set(1, 0);
    vTaskDelay(30 / portTICK_PERIOD_MS);

    /* get pos from other type */
    int pos = 0;
    switch (act_type) {
    case ACT_POS:
        int *tmp = (int *)param;
        pos = *tmp;
        break;
    case ACT_LEN:
        float *len = (float *)param;
        pos = convert_len_to_pos(*len);
        break;
    case ACT_FREQ:
        float *freq = (float *)param;
        pos = convert_freq_to_pos(*freq);
        break;
    default:
        ESP_LOGE(TAG, "act_type err: %d", act_type);
        return ESP_ERR_INVALID_ARG;
    }

    /* move stepper motor and strum */
    int stepper_motor_estimated_time = calc_stepper_motor_time_by_pos(pos);
    if (stepper_motor_estimated_time < 0) {
        return stepper_motor_estimated_time;
    }
#ifdef CONFIG_DEBUG_PRINT
    ESP_LOGI(TAG, "stepper motor action estimated time: %d ms", stepper_motor_estimated_time);
#endif
    if (stepper_motor_estimated_time <= SERVO_STRUM_PREPARE_TIME) { // stepper motor cost < servo prepare in 6v, immidiately strum

        servo_motor_action(3); // strum start

        rc = stepper_motor_action_by_pos(true, pos);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "play fail act_type: %d, pos: %d", act_type, pos);
            return rc;
        }

        // after ruler move, press the ruler
        electromagnet_set(0, POLARITY_POSITIVE);
        electromagnet_set(1, POLARITY_NEGATIVE);

        // strum: now touch the ruler)

        vTaskDelay(pdMS_TO_TICKS(SERVO_STRUM_START_2_RELEASE_TIME - stepper_motor_estimated_time));
        // strum end
    } else {
        // start timer (time alart time: stepper_motor_estimated_time - SERVO_STRUM_PREPARE_TIME)
#ifdef CONFIG_DEBUG_PRINT
        int64_t start_time = esp_timer_get_time();
        ESP_LOGI(TAG, "timer start time in: %llu ms, set timer: %d ms", start_time / 1000, stepper_motor_estimated_time - SERVO_STRUM_PREPARE_TIME);
#endif
        ESP_ERROR_CHECK(gptimer_set_raw_count(g_gptimer, 0)); // reset counter
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = (stepper_motor_estimated_time - SERVO_STRUM_PREPARE_TIME) * 1000,
        };
        ESP_ERROR_CHECK(gptimer_set_alarm_action(g_gptimer, &alarm_config));
        ESP_ERROR_CHECK(gptimer_start(g_gptimer)); // strum in timer callback

        rc = stepper_motor_action_by_pos(true, pos);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "play fail act_type: %d, pos: %d", act_type, pos);
            return rc;
        }

        // after ruler move, press the ruler
        electromagnet_set(0, POLARITY_POSITIVE);
        electromagnet_set(1, POLARITY_NEGATIVE);

        vTaskDelay(pdMS_TO_TICKS(SERVO_STRUM_START_2_RELEASE_TIME - SERVO_STRUM_PREPARE_TIME));
#ifdef CONFIG_DEBUG_PRINT
        ESP_LOGI(TAG, "timer alarm time in: %llu ms, timer cost: %llu ms", alarm_time / 1000, (alarm_time - start_time) / 1000);
#endif
    }

    // rc = stepper_motor_action_by_pos(true, pos);
    // if (rc != ESP_OK) {
    //     ESP_LOGE(TAG, "play fail act_type: %d, pos: %d", act_type, pos);
    //     return rc;
    // }

    // // press
    // electromagnet_set(0, POLARITY_POSITIVE);
    // electromagnet_set(1, POLARITY_NEGATIVE);

    // // strum
    // servo_motor_action(3);
    // vTaskDelay(pdMS_TO_TICKS(SERVO_STRUM_START_2_RELEASE_TIME));

    return rc;
}

/**
 * @brief Play a note by frequency, blocking function
 *
 * @param freq
 * @return int
 */
int play_sigle_note_by_freq(float freq)
{
    return play_single_note(ACT_FREQ, &freq);
}

int play_sigle_note_by_len(float len)
{
    return play_single_note(ACT_LEN, &len);
}

int play_sigle_note_by_pos(int pos)
{
    return play_single_note(ACT_POS, &pos);
}

int play_sigle_note_by_midi(int midi)
{
    if (midi < 0 || midi > 127) {
        ESP_LOGE(TAG, "Invalid MIDI number: %d", midi);
        return ESP_ERR_INVALID_ARG;
    }
    float freq = convert_midi_to_freq(midi);
    return play_sigle_note_by_freq(freq);
}
