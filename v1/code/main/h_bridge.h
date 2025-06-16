#ifndef __H_BRIDGE_H
#define __H_BRIDGE_H

#include <stdint.h>

extern void h_bridge_init(void);
extern void h_bridge_set(int index, int polarity);

#define BDC_RELESE_DELAY 50
#define BDC_PRESS_DELAY 60

#endif // __H_BRIDGE_H
