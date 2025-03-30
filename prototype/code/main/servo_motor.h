
#ifndef __SERVO_MOTOR_H
#define __SERVO_MOTOR_H

extern void servo_motor_init(void);
extern TaskHandle_t get_servo_motor_task_handle(void);
extern void servo_motor_action(int index);

#endif // __SERVO_MOTOR_H
