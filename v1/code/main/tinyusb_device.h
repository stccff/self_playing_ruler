#ifndef __TUSB_MIDI_H
#define __TUSB_MIDI_H

extern int tinyusb_device_init(void);
extern void set_input_midi_channel(uint8_t ch_index);

#endif // __TUSB_MIDI_H
