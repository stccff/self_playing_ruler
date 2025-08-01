
#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include <stdint.h>

#define MODE 1 // 0: full step, 1: half step, 2: 1/4 step, 3: 1/8 step, 4: 1/16 step, 5: 1/32 step
#define FULL_STEP_LEN 0.15 // mm
#define MAX_FULL_STEP 320
#define STEP_LEN (FULL_STEP_LEN / (1 << MODE))
#define MAX_STEP (MAX_FULL_STEP * (1 << MODE))
#define SCREW_BACKLASH (2 * (1 << MODE)) // step


extern void stepper_motor_init(void);

extern int stepper_motor_action_by_pos(bool is_sync, int pos);
extern void stepper_motor_async_wait_done(void);

extern int calc_stepper_motor_time_by_pos(int pos);

#endif // __STEP_MOTOR_H
