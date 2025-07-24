#ifndef __TUSB_MIDI_H
#define __TUSB_MIDI_H

extern int tinyusb_device_init(void);
extern void midi_set_input_channel(uint8_t ch_index);
extern void midi_enable_volecity(bool enable);

#endif // __TUSB_MIDI_H
