#ifndef __RULER_H
#define __RULER_H
#include <stdint.h>

extern void ruler_init(void);

extern int freq_table_init(bool force_init);
extern int freq_table_clear(void);
extern int freq_table_show(void);

// extern int ruler_action_by_freq(bool is_sync, double freq);
// extern int ruler_action_by_len(bool is_sync, double len);

extern int convert_freq_to_pos(double target_freq);
extern int convert_len_to_pos(double len);

extern void freq_table_use_formula(bool is_formula);
extern int recalculate_params(void);

#endif // __RULER_H
