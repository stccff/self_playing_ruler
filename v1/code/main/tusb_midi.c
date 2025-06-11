
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "esp_timer.h"
#include "play.h"


static const char *TAG = "midi";

/** Helper defines **/

// Interface counter
enum interface_count {
#if CFG_TUD_MIDI
    ITF_NUM_MIDI = 0,
    ITF_NUM_MIDI_STREAMING,
#endif
    ITF_COUNT
};

// USB Endpoint numbers
enum usb_endpoints {
    // Available USB Endpoints: 5 IN/OUT EPs and 1 IN EP
    EP_EMPTY = 0,
#if CFG_TUD_MIDI
    EPNUM_MIDI,
#endif
};

/** TinyUSB descriptors **/

#define TUSB_DESCRIPTOR_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_MIDI * TUD_MIDI_DESC_LEN)

/**
 * @brief String descriptor
 */
static const char* s_str_desc[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example MIDI device", // 4: MIDI
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and a MIDI interface
 */
static const uint8_t s_midi_cfg_desc[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESCRIPTOR_TOTAL_LEN, 0, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 64),
};

#if (TUD_OPT_HIGH_SPEED)
/**
 * @brief High Speed configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and a MIDI interface
 */
static const uint8_t s_midi_hs_cfg_desc[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESCRIPTOR_TOTAL_LEN, 0, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 512),
};
#endif // TUD_OPT_HIGH_SPEED

uint8_t g_input_channel = 0xff; // 0xff means all channels are active, 0~15 means channel 1~16

/**
 * @brief Set the input active midi channel
 *
 * @param ch_index 0,1~16 (0 means all channels are active)
 */
void set_input_midi_channel(uint8_t ch_index)
{
    if (ch_index > 16) {
        ESP_LOGE(TAG, "midi channel %d is invalid", ch_index);
        return;
    }
    g_input_channel = ch_index - 1;
    if (ch_index == 0x0) {
        ESP_LOGI(TAG, "set all midi channels active");
    } else {
        ESP_LOGI(TAG, "set midi channel %d active", ch_index);
    }
}


static void midi_task_read_example(void *arg)
{
    int rc = ESP_OK;
    // The MIDI interface always creates input and output port/jack descriptors
    // regardless of these being used or not. Therefore incoming traffic should be read
    // (possibly just discarded) to avoid the sender blocking in IO
    uint8_t packet[4];
    bool read = false;
    for (;;) {
        while (tud_midi_available()) { // check if has data to read
            read = tud_midi_packet_read(packet);
            if (read) {
                // ESP_LOGI(TAG, "Read - Time (ms since boot): %lld, Data: %02hhX %02hhX %02hhX %02hhX",
                //          esp_timer_get_time(), packet[0], packet[1], packet[2], packet[3]);

                // parse MIDI data
                uint8_t cable = packet[0] >> 4;
                uint8_t code_index = packet[0] & 0x0F;
                uint8_t status = packet[1] & 0xF0;
                uint8_t channel = packet[1] & 0x0F;
                uint8_t data1 = packet[2];
                uint8_t data2 = packet[3];

                // only deal whith Note On/Off
                if (code_index == 0x9 && data2 != 0) {
                    ESP_LOGI(TAG, "Note On: channel=%d note=%d velocity=%d", channel + 1, data1, data2);
                    if (g_input_channel == 0xff || channel == g_input_channel) {
                        if (data2 > 0) {
                            rc = play_sigle_note_by_midi(data1);
                            if (rc != ESP_OK) {
                                ESP_LOGE(TAG, "play_sigle_note_by_midi fail!");
                            }
                        }
                    }
                } else if (code_index == 0x8 || (code_index == 0x9 && data2 == 0)) {
                    ESP_LOGI(TAG, "Note Off: channel=%d note=%d velocity=%d", channel + 1, data1, data2);
                } else {
                    ESP_LOGI(TAG, "MIDI: cable=%d code_index=%02X status=%02X channel=%d data1=%d data2=%d",
                             cable, code_index, status, channel + 1, data1, data2);
                }
            }
        }
          vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void midi_init(void)
{
    ESP_LOGI(TAG, "USB initialization");

    tinyusb_config_t const tusb_cfg = {
        .device_descriptor = NULL, // If device_descriptor is NULL, tinyusb_driver_install() will use Kconfig
        .string_descriptor = s_str_desc,
        .string_descriptor_count = sizeof(s_str_desc) / sizeof(s_str_desc[0]),
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = s_midi_cfg_desc, // HID configuration descriptor for full-speed and high-speed are the same
        .hs_configuration_descriptor = s_midi_hs_cfg_desc,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = s_midi_cfg_desc,
#endif // TUD_OPT_HIGH_SPEED
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB initialization DONE");


    // Read received MIDI packets
    ESP_LOGI(TAG, "MIDI read task init");
    xTaskCreate(midi_task_read_example, "midi_task_read_example", 4 * 1024, NULL, 10, NULL);
}
