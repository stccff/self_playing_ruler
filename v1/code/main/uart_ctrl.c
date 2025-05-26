#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "servo_motor.h"
#include "step_motor.h"
#include "uart_ctrl.h"
#include "math.h"
#include "note_decode.h"
#include "string.h"
#include "stdlib.h"
#include "esp_timer.h"

// uart configurations
#define ECHO_TEST_TXD (43)
#define ECHO_TEST_RXD (44)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (0)
#define ECHO_UART_BAUD_RATE     (115200)
#define ECHO_TASK_STACK_SIZE    (1024 * 10)


static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)


// double k = 229049.0652;
// #define TOTAL_LENGTH 44.5   // mm
// #define TOTAL_POS 300       // steps
// #define LENGHT_MIN 13.5     // mm
// #define LENGHT_MAX 57.5     // mm


/* ***************************************************************************************************************** */
/*                                           function prototype                                                      */
/* ***************************************************************************************************************** */


static int do_set(char *cmd)
{
    int base, scale;
    if (sscanf(cmd, "set %d %d", &base, &scale) != 2) {
        ESP_LOGE(TAG, "Invalid set command: %s", cmd);
        return ESP_ERR_INVALID_ARG;
    }
    return set_base_and_scale(base, scale);
}

static int do_play(char *cmd)
{
    int midi = parse_simple_note_to_midi(cmd + strlen("p "));
    float freq = convert_midi_to_freq(midi);
    // ESP_LOGI(TAG, "midi = %d, freq = %f", midi, freq);

    // fret up
    servo_motor_action(1);
    // move to the position
    stepper_motor_action(freq);
    // fret down, max octave(eg. lowest note E2~E3) cost 150ms at SPEDD_Hz=700
    servo_motor_action(2);
    // strum
    servo_motor_action(3);

    return ESP_OK;
}

static int do_test(char *cmd)
{
    char* data = cmd + strlen("test ");
    double len;
    sscanf(data, "%lf\n", &len);

    // release the ruler
    servo_motor_action(1);
    // move to the position
    int step = step_motor_action_by_len(len);
    ESP_LOGI(TAG, "len = %f, step = %d",len, step);
    // fret // max octave(eg. lowest note E2~E3) cost 150ms at SPEDD_Hz=700
    servo_motor_action(2);
    // strum
    servo_motor_action(3);

    return ESP_OK;
}


/**
 * @brief 
 * 
 * @param cmd 
 * @return int 
 */
int dispatch_uart_cmd(char *cmd)
{
    int rc = ESP_OK;
    if (strncmp(cmd, "set", strlen("set")) == 0) {
        rc = do_set(cmd);
    } else if (strncmp(cmd, "p", strlen("p")) == 0) {
        rc = do_play(cmd);
    } else if (strncmp(cmd, "test", strlen("test")) == 0) {
        rc = do_test(cmd);
    } else {
        ESP_LOGE(TAG, "Invalid command: %s", cmd);
        return ESP_FAIL;
    }
    return rc;
}


static void uart_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    char *data = (char *) malloc(BUF_SIZE);

    int total_len = 0;
    uint32_t delay = 10; // ms
    uint32_t no_data_num = 0;
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, &data[total_len], (BUF_SIZE - total_len - 1), delay / portTICK_PERIOD_MS);
        if (len > 0) {
            total_len += len;
            if (data[total_len - 1] != '\n' && data[total_len - 1] != '\r') {
                continue;
            } else {
                data[total_len] = '\0';
                total_len = 0;
            }
            no_data_num = 0;
        } else {
            no_data_num++;
            if (no_data_num == 10 * 1000 / delay) {
                servo_motor_action(1); // avoid stalling
            }
            continue;
        }
        // ESP_LOGI(TAG, "data = %s", data);

        int64_t start = esp_timer_get_time();
        (void)dispatch_uart_cmd(data);
        int64_t end = esp_timer_get_time();
        ESP_LOGI(TAG, "UART command execution time: %lld ms", (end - start) / 1000);
    }   
}

void uart_init(void)
{
    xTaskCreate(uart_task, "uart_task", ECHO_TASK_STACK_SIZE, NULL, 5, NULL);
}

