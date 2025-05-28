#ifndef __SIGNAL_FFT_H
#define __SIGNAL_FFT_H

#include <stdint.h>
#include <stddef.h>

extern int fft_init(int fft_size);
extern void do_fft(float *buff, size_t size);

#endif // __SIGNAL_FFT_H
