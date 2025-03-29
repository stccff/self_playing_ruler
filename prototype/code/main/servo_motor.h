
#ifndef __SERVO_MOTOR_H
#define __SERVO_MOTOR_H

extern void servo_motor_init(void);
extern TaskHandle_t get_servo_motor_task_handle(void);
extern int get_strum_servo_delay(void);
extern int get_fret_servo_delay(void);

#endif // __SERVO_MOTOR_H
