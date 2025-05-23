#ifndef __NOTE_DECODE_H
#define __NOTE_DECODE_H

extern int parse_simple_note_to_midi(const char *note);
extern float convert_midi_to_freq(int midi);
extern int set_base_and_scale(int base, int scale);

#endif // __NOTE_DECODE_H
