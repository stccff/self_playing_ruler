
#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include <stdint.h>

extern void init_nvs_for_freq_table(void);
extern int freq_table_init(bool force_init);
extern int freq_table_clear(void);

extern void stepper_motor_init(void);

extern int stepper_motor_action_by_pos(int pos);
extern int stepper_motor_action_by_freq(double freq);
extern int stepper_motor_action_by_len(double len);

#endif // __STEP_MOTOR_H
