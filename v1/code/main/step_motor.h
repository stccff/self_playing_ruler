
#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H


extern void step_motor_init(void);
extern TaskHandle_t get_step_motor_task_handle(void);
extern int step_motor_action(double freq);
extern int step_motor_action_by_len(double len);

#endif // __STEP_MOTOR_H
