#ifndef __DIGITAL_MIC_H
#define __DIGITAL_MIC_H

#include <stdint.h>

extern void i2s_driver_init(void);
extern int sound_fft_init(int fft_size);
extern int get_sound_frequency(float min, float max, float *freq, bool print);
extern void mic_enable(bool enable);
extern void mic_reconfig_sample_rate(int sample_rate);
extern int read_mic_data(uint8_t *i2s_buff, float *out_buff, size_t sample_num, bool is_dma_clear);

#endif // __DIGITAL_MIC_H
