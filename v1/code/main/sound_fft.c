
#include "sound_fft.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "esp_dsp.h"

/* ***************************************************************************************************************** */
/*                                               macro define                                                        */
/* ***************************************************************************************************************** */
#define TAG "sound_fft"
/* ***************************************************************************************************************** */
/*                                               struct define                                                       */
/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */
/*                                               global variable                                                     */
/* ***************************************************************************************************************** */

static bool is_power_of_four(int n)
{
    if (n <= 0) {
        return false;
    }
    while (n % 4 == 0) {
        n /= 4;
    }
    return n == 1;
}

static int fft_init(int fft_size, float **wind)
{
    int ret = ESP_OK;
    // fft initialization

    ret = dsps_fft2r_init_fc32(NULL, fft_size >> 1);
    if (ret == ESP_ERR_DSP_REINITIALIZED) {
        dsps_fft2r_deinit_fc32();
        ret = dsps_fft2r_init_fc32(NULL, fft_size >> 1);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Not possible to initialize FFT2R. Error = %x", ret);
        return ret;
    }
    
    ret = dsps_fft4r_init_fc32(NULL, fft_size >> 1);
    if (ret == ESP_ERR_DSP_REINITIALIZED) {
        dsps_fft4r_deinit_fc32();
        ret = dsps_fft4r_init_fc32(NULL, fft_size >> 1);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Not possible to initialize FFT4R. Error = %x", ret);
        return ret;
    }

    // Generate hann window
    if (*wind != NULL) {
        free(*wind);
    }
    *wind = (float *)malloc(fft_size * sizeof(float));
    if (*wind == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for window");
        return ESP_ERR_NO_MEM;
    }
    dsps_wind_hann_f32(*wind, fft_size);
}

static void do_fft(float *buff, float *wind, size_t size)
{
        // add window
        for (int i = 0 ; i < size ; i++) {
            buff[i] = buff[i] * wind[i];
        }

        if (is_power_of_four(size)) {
            // FFT Radix-4
            dsps_fft4r_fc32(buff, size);
            // Bit reverse
            dsps_bit_rev4r_fc32(buff, size);
            // Convert one complex vector with length FFT_SIZE/2 to one real spectrum vector with length FFT_SIZE/2
            dsps_cplx2real_fc32(buff, size);
        } else {
            // FFT Radix-2
            dsps_fft2r_fc32(buff, size >> 1);
            // Bit reverse
            dsps_bit_rev2r_fc32(buff, size >> 1);
            // Convert one complex vector with length FFT_SIZE/2 to one real spectrum vector with length FFT_SIZE/2
            dsps_cplx2real_fc32(buff, size >> 1);
        }
        
        // amplitude correction factor
        float hann_correction_factor = 2.0;
        float fft_normalization_factor = 2.0 / size;
    
        for(int i=0; i < size/2; i++) {
            float real = buff[i*2];
            float imag = buff[i*2+1];
            buff[i] = sqrtf(real*real + imag*imag) * fft_normalization_factor * hann_correction_factor;
        }
    
        // // calculate 
        // // find the peak value
        // size_t k = 0;
        // float max = buff[0];
        // for (size_t i = 1; i < size/2; i++) {
        //     if (buff[i] > max) {
        //         max = buff[i];
        //         k = i;
        //     }
        // }
        // // Tri-node parabolic interpolation
        // float delt = (buff[k+1] - buff[k-1]) / (4*buff[k] - 2*buff[k-1] - 2*buff[k+1]);
        // float freq = (k + delt) * EXAMPLE_SAMPLE_RATE / size;
        // float freq_max = (float)k * EXAMPLE_SAMPLE_RATE / size;
        // ESP_LOGI(TAG, "max freq = %f, vertex freq = %f", freq_max, freq);
}
