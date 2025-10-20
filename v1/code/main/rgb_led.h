#ifndef __RGB_LED_H__
#define __RGB_LED_H__

#include <stdint.h>

extern void rgb_led_init(void);
extern int devkitc_rgb_led_set_hsv(uint16_t h, float s, float v);


#endif // __RGB_LED_H__
