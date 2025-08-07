#ifndef __NOTE_DECODE_H
#define __NOTE_DECODE_H

#include <stdint.h>
#include <stdbool.h>

extern int parse_simple_note_to_midi(const char *note);
extern float convert_midi_to_freq(int midi);
extern int set_base_and_scale(int base, int scale);
extern int find_midi_idx_by_freq(float freq, bool ret_greater);

#endif // __NOTE_DECODE_H
