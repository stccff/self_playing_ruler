#ifndef __ELECTROMAGNET_H
#define __ELECTROMAGNET_H

#include <stdint.h>

extern void electromagnet_init(void);
extern void electromagnet_set(int index, int polarity);

#endif // __ELECTROMAGNET_H
