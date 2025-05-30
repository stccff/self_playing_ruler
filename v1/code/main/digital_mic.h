#ifndef __DIGITAL_MIC_H
#define __DIGITAL_MIC_H

#include <stdint.h>

extern void i2s_driver_init(void);
extern int sound_fft_init(float fft_size);
extern int get_sound_frequency(float *freq, bool print);

#endif // __DIGITAL_MIC_H
