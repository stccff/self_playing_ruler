#ifndef __PLAY_H
#define __PLAY_H
#include <stdint.h>

extern int play_sigle_note_by_pos(int pos);
extern int play_sigle_note_by_freq(float freq);
extern int play_sigle_note_by_len(float len);
extern int play_sigle_note_by_midi(int midi);
extern void play_timer_init(void);

#endif // __PLAY_H
