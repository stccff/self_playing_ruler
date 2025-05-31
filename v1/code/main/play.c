#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "play.h"
#include "servo_motor.h"
#include "step_motor.h"
#include "electromagnet.h"
#include "note_decode.h"

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
/* ***************************************************************************************************************** */
/*                                               global variable                                                     */
/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */
/*                                              function prototype                                                   */
/* ***************************************************************************************************************** */

static int play_single_note(int (*stepper_act_func)(double), float param)
{
    // release the ruler
    electromagnet_set(0, POLARITY_NEGATIVE);
    vTaskDelay(10 / portTICK_PERIOD_MS); // TODO: is this delay needed?
    electromagnet_set(0, 0);
    electromagnet_set(1, 0);

    // move to the position
    stepper_act_func(param);

    // fret
    electromagnet_set(0, POLARITY_POSITIVE);
    electromagnet_set(1, POLARITY_NEGATIVE);

    // strum
    servo_motor_action(3);

    return ESP_OK;
}

/**
 * @brief Play a note by frequency, blocking function
 *
 * @param freq
 * @return int
 */
int play_sigle_note_by_freq(float freq)
{
    return play_single_note(stepper_motor_action_by_freq, freq);
}

int play_sigle_note_by_len(float len)
{
    return play_single_note(stepper_motor_action_by_len, len);
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
