/**
 * @file electromagnet.c
 * @author your name (you@domain.com)
 * @brief Electromagnet control module, use H bridege chip.
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
#include "electromagnet.h"
#include "gpio_pin_config.h"

/* ***************************************************************************************************************** */
/*                                               macro define                                                        */
/* ***************************************************************************************************************** */
#define TAG "electromagnet"

/* ***************************************************************************************************************** */
/*                                               struct define                                                       */
/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */
/*                                               global variable                                                     */
/* ***************************************************************************************************************** */
int g_e_magnet_io_cfg[2][2] = {
    {MAGNET_TOP_A_GPIO, MAGNET_TOP_B_GPIO},   // Top electromagnet
    {MAGNET_BOTTOM_A_GPIO, MAGNET_BOTTOM_B_GPIO} // Bottom electromagnet
};

void electromagnet_init(void)
{
    ESP_LOGI(TAG, "Initialize e-magnet GPIO");
    gpio_config_t magnet_io_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << MAGNET_TOP_A_GPIO | 1ULL << MAGNET_TOP_B_GPIO |
                        1ULL << MAGNET_BOTTOM_A_GPIO | 1ULL << MAGNET_BOTTOM_B_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&magnet_io_cfg));

    electromagnet_set(0, 0);
    electromagnet_set(1, 0); // off
}

void electromagnet_set(int index, int polarity)
{
    if (index < 0 || index >= sizeof(g_e_magnet_io_cfg) / sizeof(g_e_magnet_io_cfg[0])) {
        ESP_LOGE(TAG, "Invalid index: %d", index);
        return;
    }

    if (polarity > 0) {
        // Set the electromagnet to the specified polarity
        ESP_ERROR_CHECK(gpio_set_level(g_e_magnet_io_cfg[index][0], 0));
        ESP_ERROR_CHECK(gpio_set_level(g_e_magnet_io_cfg[index][1], 1));
    } else if (polarity < 0) {
        // Set the electromagnet to the opposite polarity
        ESP_ERROR_CHECK(gpio_set_level(g_e_magnet_io_cfg[index][0], 1));
        ESP_ERROR_CHECK(gpio_set_level(g_e_magnet_io_cfg[index][1], 0));
    } else {
        // Turn off electromagnet
        ESP_ERROR_CHECK(gpio_set_level(g_e_magnet_io_cfg[index][0], 0));
        ESP_ERROR_CHECK(gpio_set_level(g_e_magnet_io_cfg[index][1], 0));
    }
}
