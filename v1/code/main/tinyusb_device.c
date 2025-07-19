/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/usb_phy.h"
#include "tusb.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gptimer.h"
#include "play.h"
#include "digital_mic.h"
#include "tinyusb_device.h"


#define TAG "tinyusb"

#define USBD_STACK_SIZE 4096
#define BLINKY_STACK_SIZE 4096
#define AUDIO_STACK_SIZE 4096
#define MIDI_STACK_SIZE 4096


//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
#define AUDIO_SAMPLE_RATE CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

static bool g_audio_streaming_enabled = false;


// Audio controls
// Current states
bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];       // +1 for master channel 0
uint16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; // +1 for master channel 0
uint32_t sampFreq;
uint8_t clkValid;

// Range states
audio_control_range_2_n_t(1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; // Volume range state
audio_control_range_4_n_t(1) sampleFreqRng;                                     // Sample frequency range state

#if CFG_TUD_AUDIO_ENABLE_ENCODING
// Audio test data, each buffer contains 2 channels, buffer[0] for CH0-1, buffer[1] for CH1-2
// uint16_t i2s_dummy_buffer[CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO][CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX * CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE / 1000 / CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO];
#else
uint8_t i2s_dummy_buffer[CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX * AUDIO_SAMPLE_RATE / 1000];
#endif


static usb_phy_handle_t phy_hdl;
SemaphoreHandle_t binary_sem;


static void led_blinking_task(void *param);
static void usb_device_task(void *param);
static void audio_task(void *param);
static void midi_task(void *param);

static void usb_phy_init(void)
{
    // Configure USB PHY
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT,
    };
    usb_new_phy(&phy_conf, &phy_hdl);
}

static bool example_timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    // 处理事件回调的一般流程：
    // 1. 从 user_ctx 中拿到用户上下文数据（需事先从 gptimer_register_event_callbacks 中传入）
    // 2. 从 edata 中获取警报事件数据，比如 edata->count_value
    // 3. 执行用户自定义操作
    // 4. 返回上述操作期间是否有高优先级的任务被唤醒了，以便通知调度器做切换任务

    BaseType_t high_task_awoken = pdFALSE;
    xSemaphoreGiveFromISR(binary_sem, &high_task_awoken);

    return high_task_awoken;
}

static void create_timer(void)
{
    binary_sem = xSemaphoreCreateBinary(); // 初始化二值信号量
    ESP_ERROR_CHECK(binary_sem == NULL);

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // 选择默认的时钟源
        .direction = GPTIMER_COUNT_UP,      // 计数方向为向上计数
        .resolution_hz = 1 * 1000 * 1000,   // 分辨率为 1 MHz，即 1 次滴答为 1 微秒
    };
    // 创建定时器实例
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,      // 当警报事件发生时，定时器会自动重载到 0
        .alarm_count = 1000, // 设置实际的警报周期，因为分辨率是 1us，所以 1000 代表 1ms
        .flags.auto_reload_on_alarm = true, // 使能自动重载功能
    };
    // 设置定时器的警报动作
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = example_timer_on_alarm_cb, // 当警报事件发生时，调用用户回调函数
    };
    // 注册定时器事件回调函数，允许携带用户上下文
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    // 使能定时器
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    // 启动定时器
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}


void tud_sof_cb(uint32_t frame_count)
{
    printf("%lu\n", frame_count);
    // 每1ms调用一次
    // 在这里采集/发送音频数据
    // tud_audio_write_support_ff(0, i2s_dummy_buffer[0], AUDIO_SAMPLE_RATE / 1000 * CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_CHANNEL_PER_FIFO_TX);
}

int tinyusb_device_init(void)
{
    ESP_LOGI(TAG, "USB Initializing...\n");

    usb_phy_init();

    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO
    };
    bool usb_init = tusb_init(BOARD_TUD_RHPORT, &dev_init);
    if (!usb_init) {
        ESP_LOGE(TAG, "USB Device Stack Init Fail");
        return ESP_FAIL;
    }

    // Init values
    sampFreq = AUDIO_SAMPLE_RATE;
    clkValid = 1;

    sampleFreqRng.wNumSubRanges = 1;
    sampleFreqRng.subrange[0].bMin = AUDIO_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bMax = AUDIO_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bRes = 0;

    // tud_sof_cb_enable(true);
    // xTaskCreate(led_blinking_task, "blinky", BLINKY_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    create_timer();
    xTaskCreate(audio_task, "audio", AUDIO_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(midi_task, "midi", MIDI_STACK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL);

    return ESP_OK;
}

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
static void usb_device_task(void *param)
{
    (void)param;
    // RTOS forever loop
    while (1) {
        // tinyusb device task
        tud_task();
    }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
    blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+
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


static void midi_task(void *arg) // TODO: move the detail to other c file
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
                            rc = play_single_note_by_midi(data1);
                            if (rc != ESP_OK) {
                                ESP_LOGE(TAG, "play_single_note_by_midi fail!");
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

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

static void audio_task(void *param)
{
    (void)param;
    // Yet to be filled - e.g. read audio from I2S buffer.
    // Here we simulate a I2S receive callback every 1ms.
    while (1)
    {
        // vTaskDelay(1);
        if (xSemaphoreTake(binary_sem, portMAX_DELAY)) {
            if (g_audio_streaming_enabled) {
#if CFG_TUD_AUDIO_ENABLE_ENCODING
#else
                int sample_num = AUDIO_SAMPLE_RATE / 1000;
                int rc = read_mic_data(i2s_dummy_buffer, NULL, sample_num, false);
                if (rc != ESP_OK) {
                    ESP_LOGE(TAG, "read_mic_data fail! rc=%d", rc);
                } else {
                    // Write the audio data to the USB audio interface
                    tud_audio_write(i2s_dummy_buffer, sample_num * CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX);
                }
#endif
            }
        }

    }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    (void)rhport;
    (void)pBuff;

    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t ep = TU_U16_LOW(p_request->wIndex);

    (void)channelNum;
    (void)ctrlSel;
    (void)ep;

    return false; // Yet not implemented
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    (void)rhport;
    (void)pBuff;

    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);

    (void)channelNum;
    (void)ctrlSel;
    (void)itf;

    return false; // Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    (void)rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    (void)itf;

    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // If request is for our feature unit
    if (entityID == 2)
    {
        switch (ctrlSel)
        {
        case AUDIO_FU_CTRL_MUTE:
            // Request uses format layout 1
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

            mute[channelNum] = ((audio_control_cur_1_t *)pBuff)->bCur;

            TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
            return true;

        case AUDIO_FU_CTRL_VOLUME:
            // Request uses format layout 2
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

            volume[channelNum] = ((audio_control_cur_2_t *)pBuff)->bCur;

            TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
            return true;

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }
    return false; // Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t ep = TU_U16_LOW(p_request->wIndex);

    (void)channelNum;
    (void)ctrlSel;
    (void)ep;

    return false; // Yet not implemented
}

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);

    (void)channelNum;
    (void)ctrlSel;
    (void)itf;

    return false; // Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    // uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    // Input terminal (Microphone input)
    if (entityID == 1)
    {
        switch (ctrlSel)
        {
        case AUDIO_TE_CTRL_CONNECTOR:
        {
            // The terminal connector control only has a get request with only the CUR attribute.
            audio_desc_channel_cluster_t ret;

            // Those are dummy values for now
            ret.bNrChannels = 1;
            ret.bmChannelConfig = 0;
            ret.iChannelNames = 0;

            TU_LOG2("    Get terminal connector\r\n");

            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *)&ret, sizeof(ret));
        }
        break;

            // Unknown/Unsupported control selector
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    // Feature unit
    if (entityID == 2)
    {
        switch (ctrlSel)
        {
        case AUDIO_FU_CTRL_MUTE:
            // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
            // There does not exist a range parameter block for mute
            TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
            return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);

        case AUDIO_FU_CTRL_VOLUME:
            switch (p_request->bRequest)
            {
            case AUDIO_CS_REQ_CUR:
                TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
                return tud_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));

            case AUDIO_CS_REQ_RANGE:
                TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);

                // Copy values - only for testing - better is version below
                audio_control_range_2_n_t(1)
                    ret;

                ret.wNumSubRanges = 1;
                ret.subrange[0].bMin = -90; // -90 dB
                ret.subrange[0].bMax = 90;  // +90 dB
                ret.subrange[0].bRes = 1;   // 1 dB steps

                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *)&ret, sizeof(ret));

                // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }
            break;

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    // Clock Source unit
    if (entityID == 4)
    {
        switch (ctrlSel)
        {
        case AUDIO_CS_CTRL_SAM_FREQ:
            // channelNum is always zero in this case
            switch (p_request->bRequest)
            {
            case AUDIO_CS_REQ_CUR:
                TU_LOG2("    Get Sample Freq.\r\n");
                // Buffered control transfer is needed for IN flow control to work
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));

            case AUDIO_CS_REQ_RANGE:
                TU_LOG2("    Get Sample Freq. range\r\n");
                return tud_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));

                // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }
            break;

        case AUDIO_CS_CTRL_CLK_VALID:
            // Only cur attribute exists for this request
            TU_LOG2("    Get Sample Freq. valid\r\n");
            return tud_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));

        // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    TU_LOG2("  Unsupported entity: %d\r\n", entityID);
    return false; // Yet not implemented
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)itf;
    (void)ep_in;
    (void)cur_alt_setting;

    // In read world application data flow is driven by I2S clock,
    // both tud_audio_tx_done_pre_load_cb() & tud_audio_tx_done_post_load_cb() are hardly used.
    // For example in your I2S receive callback:
    // void I2S_Rx_Callback(int channel, const void* data, uint16_t samples)
    // {
    //    tud_audio_write_support_ff(channel, data, samples * N_BYTES_PER_SAMPLE * N_CHANNEL_PER_FIFO);
    // }

    return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)n_bytes_copied;
    (void)itf;
    (void)ep_in;
    (void)cur_alt_setting;

    return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    (void)p_request;

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    uint8_t alt = TU_U16_LOW(p_request->wValue);

    if (itf == 1) {
        if (alt == 0) { // 0 means no audio streaming
            mic_enable(false);
            g_audio_streaming_enabled = false;
            ESP_LOGI(TAG, "Audio streaming disabled");
        } else {
            mic_reconfig_sample_rate(AUDIO_SAMPLE_RATE);
            mic_enable(true);
            g_audio_streaming_enabled = true;
            ESP_LOGI(TAG, "Audio streaming enabled, alt setting %d", alt);
        }
    } else {
        ESP_LOGE(TAG, "Unsupported audio interface %d", itf);
        return false; // Unsupported interface
    }
    return true;
}
///--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
static void led_blinking_task(void *param)
{
    (void)param;
    static uint32_t start_ms = 0;
    static bool led_state = false;

    while (1)
    {
        // Blink every interval ms
        vTaskDelay(blink_interval_ms / portTICK_PERIOD_MS);
        start_ms += blink_interval_ms;

        // board_led_write(led_state);
        led_state = 1 - led_state; // toggle
    }
}
