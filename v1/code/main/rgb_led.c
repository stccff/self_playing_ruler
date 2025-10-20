
#include "esp_log.h"
#include "led_strip.h"
#include "gpio_pin_config.h"
#include "rgb_led.h"

#define TAG "reg_led"
#define LED_STRIP_LED_COUNT 1
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

led_strip_handle_t g_led_strip = NULL;

void rgb_led_init(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = 0, // the memory block size used by the RMT channel. 0 is auto????
        .flags = {
            .with_dma = 1,     // Using DMA can improve performance when driving more LEDs
        }
    };

    // LED Strip object handle
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");

    (void)led_strip_clear(g_led_strip);

    return;
}

/**
 * @brief
 *
 * @param h hue 0~360
 * @param s saturation 0~100%
 * @param v value 0~100%
 * @return int
 */
int devkitc_rgb_led_set_hsv(uint16_t h, float s, float v)
{
    int rc = ESP_OK;

    if (v == 0) {
        // turn off
        rc = led_strip_clear(g_led_strip);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "led_strip_clear failed");
        }
        return rc;
    }

    rc = led_strip_set_pixel_hsv(g_led_strip, 0, h, s*255, v*255);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_set_pixel_hsv failed");
        return rc;
    }
    rc = led_strip_refresh(g_led_strip);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_refresh failed");
        return rc;
    }
    return rc;
}
