
#ifndef __SERVO_MOTOR_H
#define __SERVO_MOTOR_H

#define SERVO_STRUM_START_2_RELEASE_TIME 100 // ms, shot by high speed camera: set angle-->strum done, in +-30 degree (88 96 90 91...)
#define SERVO_STRUM_PREPARE_TIME 60 // ms, set angle-->touch to ruler (68 70 74 ...)

extern void servo_motor_init(void);
extern void servo_motor_action(int index);

#endif // __SERVO_MOTOR_H
