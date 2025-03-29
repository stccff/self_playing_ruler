/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"
#include "math.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


///////////////////////////////Change the following configurations according to your board//////////////////////////////
#define STEP_MOTOR_GPIO_EN       0
#define STEP_MOTOR_GPIO_DIR      2
#define STEP_MOTOR_GPIO_STEP     4

#define STEP_MOTOR_MODE0_PIN    40
#define STEP_MOTOR_MODE1_PIN    41
#define STEP_MOTOR_MODE2_PIN    42

#define STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 1
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

#define MODE 1 // 0: full step, 1: half step, 2: 1/4 step, 3: 1/8 step, 4: 1/16 step, 5: 1/32 step
#define SAMPLE_POINTS 500
#define SPEED_HZ (100 * pow(2, MODE))

// uart configurations
#define ECHO_TEST_TXD (43)
#define ECHO_TEST_RXD (44)

#define ECHO_UART_PORT_NUM      0
#define ECHO_UART_BAUD_RATE     115200


static const char *TAG = "step motor test";


#define BUF_SIZE (1024)



void app_main(void)
{
    ESP_LOGI(TAG, "Initialize UART");
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
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, UART_PIN_NO_CHANGE, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);


    ESP_LOGI(TAG, "Initialize  step motor");
    ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_EN |
                        1ULL << STEP_MOTOR_MODE0_PIN | 1ULL << STEP_MOTOR_MODE1_PIN | 1ULL << STEP_MOTOR_MODE2_PIN,

    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

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
    ESP_ERROR_CHECK(rmt_enable(motor_chan));

    ESP_LOGI(TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    const static uint32_t uniform_speed_hz = SPEED_HZ;

    int total_len = 0;
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, &data[total_len], (BUF_SIZE - total_len - 1), 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            total_len += len;
            if (data[total_len - 1] != '\n' && data[total_len - 1] != '\r') {
                continue;
            } else {
                data[total_len] = '\0';
                total_len = 0;
            }
            
            int step = 0;
            sscanf((char *)data, "%d", &step);
            ESP_LOGI(TAG, "step = %d", step);

            // uniform phase
            gpio_set_level(STEP_MOTOR_GPIO_EN, 0); // enable motor
            
            gpio_set_level(STEP_MOTOR_GPIO_DIR, (step > 0) ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE); // set direction
            tx_config.loop_count = abs(step);
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(STEP_MOTOR_GPIO_EN, 1); // disable motor
        }
    }
}

