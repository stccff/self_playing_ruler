#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "digital_mic.h"
#include "signal_fft.h"
#include "gpio_pin_config.h"
#include "math.h"

/* ***************************************************************************************************************** */
/*                                               macro define                                                        */
/* ***************************************************************************************************************** */
#define TAG "microphone"

// i2s configurations
static i2s_chan_handle_t rx_handle = NULL;
/* Sample configurations */
#define EXAMPLE_SAMPLE_RATE (8000)
#define EXAMPLE_BIT_WIDTH   I2S_SLOT_BIT_WIDTH_24BIT
#define EXAMPLE_SLOT_WIDTH  I2S_SLOT_BIT_WIDTH_32BIT
#define EXAMPLE_DMA_FRAME_NUM 384
#define EXAMPLE_DMA_DESC_NUM 6
#define ONCE_READ_SAMPLE    (256)
#define INMP441_STANDBY_CLK (1 << 14)
#define INMP441_STANDBY_BYTES (INMP441_STANDBY_CLK / (EXAMPLE_SLOT_WIDTH * 2) * EXAMPLE_BIT_WIDTH / 8) // standby invalid data(each period has 2 slot, only 1 slot is valid in mono mode)
#define SAMPLE_MAX_VALUE ((1 << (EXAMPLE_BIT_WIDTH - 1)) - 1)

/* ***************************************************************************************************************** */
/*                                               struct define                                                       */
/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */
/*                                               global variable                                                     */
/* ***************************************************************************************************************** */
int g_fft_size = 0; // global variable for fft size, used in do_fft()
uint8_t *g_i2s_buff = NULL;
float *g_fft_buff = NULL;


void i2s_driver_init(void)
{
    ESP_LOGI(TAG, "i2s driver init");
    // i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_AUTO,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = EXAMPLE_DMA_DESC_NUM,
        .dma_frame_num = EXAMPLE_DMA_FRAME_NUM,
        .auto_clear_after_cb = false,
        .auto_clear_before_cb = false,
        .allow_pd = false,
        .intr_priority = 0,
    };

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));
    i2s_std_config_t std_cfg = {
        // .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        .clk_cfg = {
            .sample_rate_hz = EXAMPLE_SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_384,
        },
        // .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(EXAMPLE_BIT_WIDTH, I2S_SLOT_MODE_MONO),
        .slot_cfg = {
            .data_bit_width = EXAMPLE_BIT_WIDTH,
            .slot_bit_width = EXAMPLE_SLOT_WIDTH,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = EXAMPLE_SLOT_WIDTH,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false, // is the output data endian??
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = GPIO_NUM_NC,
            .din = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    return;
}

static int read_mic_data(uint8_t *i2s_buff, float *fft_buff, size_t signal_len, bool is_dma_clear)
{
    int rc = ESP_OK;
    size_t bytes_read = 0;
    size_t date_bytes = EXAMPLE_BIT_WIDTH / 8;
    size_t bytes_to_read = signal_len * date_bytes;

    // clear the dma buffer
    if (is_dma_clear) {
        // size_t standby_bytes = INMP441_STANDBY_BYTES;
        size_t dma_buffer_bytes = EXAMPLE_DMA_DESC_NUM * EXAMPLE_DMA_FRAME_NUM * EXAMPLE_BIT_WIDTH / 8; // dma legacy data, clear dma // TODO: if not full
        size_t skip_bytes = dma_buffer_bytes;
        uint8_t *skip_buff = (uint8_t *)malloc(skip_bytes);
        if (skip_buff == NULL) {
            ESP_LOGE(TAG, "malloc failed");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "read skip data: %d", skip_bytes);
        rc = i2s_channel_read(rx_handle, skip_buff, skip_bytes, &bytes_read, 1000);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "Read Failed!, rc=%d, bytes_read=%d", rc, bytes_read);
            free(skip_buff);
            return rc;
        }
        free(skip_buff);
    }

    // read the mic data
    // ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    rc = i2s_channel_read(rx_handle, i2s_buff, bytes_to_read, &bytes_read, 1000);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Read Failed!, rc=%d, bytes_read=%d", rc, bytes_read);
        return rc;
    }

    for (size_t i = 0; i < bytes_to_read / (EXAMPLE_BIT_WIDTH / 8); i++) {
        int tmp = (i2s_buff[i*date_bytes+2] << 16) | (i2s_buff[i*date_bytes+1] << 8) | i2s_buff[i*date_bytes];
        // printf("%d\n", (tmp << 8) >> 8);
        fft_buff[i] = (tmp << 8) >> 8; // add sign extension
    }

    return rc;
}

/**
 * @brief Initialize the FFT size and allocate memory for I2S and FFT buffers.
 *
 * @param fft_size
 * @return int
 */
int sound_fft_init(int fft_size)
{
    int rc = ESP_OK;
    if (fft_size != g_fft_size) {
        rc = fft_init(fft_size);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "FFT initialization failed with error: %d", rc);
            return rc;
        }

        if (g_i2s_buff != NULL) {
            free(g_i2s_buff);
            g_i2s_buff = NULL;
        }
        if (g_fft_buff != NULL) {
            free(g_fft_buff);
            g_fft_buff = NULL;
        }

        g_i2s_buff = (uint8_t *)malloc(fft_size * EXAMPLE_BIT_WIDTH / 8);
        if (g_i2s_buff == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for I2S buffer");
            return ESP_ERR_NO_MEM;
        }

        g_fft_buff = (float *)aligned_alloc(16, fft_size * sizeof(float));
        if (g_fft_buff == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for FFT buffer");
            free(g_i2s_buff);
            g_i2s_buff = NULL;
            return ESP_ERR_NO_MEM;
        }

        g_fft_size = fft_size;
    }
    return rc;
}

/**
 * @brief Get the sound frequency, before calling this function, you should call sound_fft_init() to initialize the FFT size.
 *
 * @param min
 * @param max
 * @param freq [out] pointer to store the calculated frequency
 * @param print
 * @return int
 */
int get_sound_frequency(float min, float max, float *freq, bool print)
{
    int rc = ESP_OK;

    if (min >= max || freq == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    rc = read_mic_data(g_i2s_buff, g_fft_buff, g_fft_size, true);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read microphone data with error: %d", rc);
        return rc;
    }
    // fft
    do_fft(g_fft_buff, g_fft_size);

    // calculate frequency
    // find the peak value

    float *buff = g_fft_buff;
    size_t size = g_fft_size;
    int freq_min_index = (int)(min / ((float)EXAMPLE_SAMPLE_RATE / (float)size));
    int freq_max_index = (int)(max / ((float)EXAMPLE_SAMPLE_RATE / (float)size));
    if (freq_max_index >= size / 2) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t k = 0;
    buff[k] = 0;
    if (print) {
        ESP_LOGI(TAG, "find in [%f -- %f]", min, max);
    }
    // for (size_t i = 1; i < size / 2; i++) {
    for (size_t i = freq_min_index; i <= freq_max_index; i++) {
        // printf("%f\n", buff[i]);
        if (buff[i] > buff[k]) {
            k = i;
        }
    }
    if (k == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    // Tri-node parabolic interpolation
    if (buff[k] < buff[k-1] || buff[k] < buff[k+1]) {
        if (print) {
            ESP_LOGE(TAG, "max value is not in the middle of three nodes, k = %d, [%f %f %f]", k, buff[k-1], buff[k], buff[k+1]);
        }
        return ESP_ERR_NOT_FOUND;
    }
    float delt = (buff[k+1] - buff[k-1]) / (4*buff[k] - 2*buff[k-1] - 2*buff[k+1]);
    float vertex = (k + delt) * EXAMPLE_SAMPLE_RATE / size;
    float freq_max = (float)k * EXAMPLE_SAMPLE_RATE / size;
    if (print) {
        ESP_LOGI(TAG, "max freq = %f, vertex freq = %f", freq_max, vertex);
        ESP_LOGD(TAG, "k = %d, delt = %f, [%.2f %.2f %.2f]", k, delt, buff[k-1], buff[k], buff[k+1]);
    }
    *freq = vertex;

    return rc;
}
