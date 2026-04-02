
#ifndef __SERVO_MOTOR_H
#define __SERVO_MOTOR_H

// start            touch       release         complete
// |------------------|----------|------------------|
//          60ms          40ms          56ms
#define SERVO_STRUM_START_2_RELEASE_TIME 100 // ms, shot by high speed camera: set angle-->strum done, in +-30 degree (88 96 90 91...)
#define SERVO_STRUM_PREPARE_TIME 60 // ms, set angle-->touch to ruler (68 70 74 ...)
#define SERVO_STRUM_FULL_TIME 156

extern void servo_motor_init(void);
extern void servo_motor_action(int act_idx);
extern int servo_set_angle(int servo_idx, float angle);
extern int servo_get_curr_angle(int servo_idx, float *angle);
extern int servo_set_offset_angle(int servo_idx, float offset_angle);
extern int servo_offset_calibration(void);
extern void servo_strum_iram_without_fpu(void);

#endif // __SERVO_MOTOR_H
