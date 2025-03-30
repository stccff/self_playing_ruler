
#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H


extern void step_motor_init(void);
extern TaskHandle_t get_step_motor_task_handle(void);
extern void step_motor_action(double freq);

#endif // __STEP_MOTOR_H
