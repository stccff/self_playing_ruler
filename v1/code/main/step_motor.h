
#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include <stdint.h>

extern int freq_table_init(bool force_init);
extern int freq_table_clear(void);
extern int freq_table_show(void);

extern void stepper_motor_init(void);

extern int stepper_motor_action_by_pos(bool is_sync, int pos);
extern int stepper_motor_action_by_freq(bool is_sync, double freq);
extern int stepper_motor_action_by_len(bool is_sync, double len);
extern void stepper_motor_async_wait_done(void);

extern int calc_stepper_motor_time_by_pos(int pos);

extern int convert_freq_to_pos(double target_freq);
extern int convert_len_to_pos(double len);

#endif // __STEP_MOTOR_H
