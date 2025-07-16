/**
 * @file h_bridge.c
 * @author your name (you@domain.com)
 * @brief H bridege chip.
 * @version 0.1
 * @date 2025-05-26
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "h_bridge.h"
#include "gpio_pin_config.h"

/* ***************************************************************************************************************** */
/*                                               macro define                                                        */
/* ***************************************************************************************************************** */
#define TAG "h-bridge"

/* ***************************************************************************************************************** */
/*                                               struct define                                                       */
/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */
/*                                               global variable                                                     */
/* ***************************************************************************************************************** */
int g_hbridge_io_cfg[2][2] = {
    {MOTOR_B_1A_IO, MOTOR_B_1B_IO},
    {MOTOR_A_1A_IO, MOTOR_A_1B_IO},
};

void h_bridge_init(void)
{
    ESP_LOGI(TAG, "Initialize e-magnet GPIO");
    gpio_config_t magnet_io_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << MOTOR_B_1A_IO | 1ULL << MOTOR_B_1B_IO |
                        1ULL << MOTOR_A_1A_IO | 1ULL << MOTOR_A_1B_IO,
    };
    ESP_ERROR_CHECK(gpio_config(&magnet_io_cfg));

    for (int i = 0; i < sizeof(g_hbridge_io_cfg) / sizeof(g_hbridge_io_cfg[0]); i++) {
        h_bridge_set(i, 0); // turn off all h-bridges
    }

#ifdef CONFIG_HW_B_VER_1_0 // TODO: abstract this to bdc.c
    h_bridge_set(1, -1); // press
    vTaskDelay(pdMS_TO_TICKS(100));
    h_bridge_set(1, 0); // off

    for (int i = 0; i < 2; i++) {
        h_bridge_set(1, 1); // release
        vTaskDelay(pdMS_TO_TICKS(BDC_RELESE_DELAY));
        h_bridge_set(1, 0); // off
    }

    for (int i = 0; i < 3; i++) {
        h_bridge_set(1, -1); // press
        vTaskDelay(pdMS_TO_TICKS(BDC_PRESS_DELAY));
        h_bridge_set(1, 0); // off

        vTaskDelay(pdMS_TO_TICKS(100));

        h_bridge_set(1, 1); // release
        vTaskDelay(pdMS_TO_TICKS(BDC_RELESE_DELAY));
        h_bridge_set(1, 0); // off
    }
#endif // CONFIG_HW_B_VER_1_0

    ESP_LOGI(TAG, "H-bridge GPIO initialized");
}

void h_bridge_set(int index, int polarity)
{
    if (index < 0 || index >= sizeof(g_hbridge_io_cfg) / sizeof(g_hbridge_io_cfg[0])) {
        ESP_LOGE(TAG, "Invalid index: %d", index);
        return;
    }

    if (polarity > 0) {
        // Set the electromagnet to the specified polarity
        ESP_ERROR_CHECK(gpio_set_level(g_hbridge_io_cfg[index][0], 0));
        ESP_ERROR_CHECK(gpio_set_level(g_hbridge_io_cfg[index][1], 1));
    } else if (polarity < 0) {
        // Set the electromagnet to the opposite polarity
        ESP_ERROR_CHECK(gpio_set_level(g_hbridge_io_cfg[index][0], 1));
        ESP_ERROR_CHECK(gpio_set_level(g_hbridge_io_cfg[index][1], 0));
    } else {
        // Turn off electromagnet
        ESP_ERROR_CHECK(gpio_set_level(g_hbridge_io_cfg[index][0], 0));
        ESP_ERROR_CHECK(gpio_set_level(g_hbridge_io_cfg[index][1], 0));
    }
}
