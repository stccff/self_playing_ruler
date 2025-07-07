#ifndef __GPIO_PIN_CONFIG_H__
#define __GPIO_PIN_CONFIG_H__

#include <stdint.h>

/* GPIO 19 20 reserved for usb D+ D- */
/* GPIO 48 reserved for RGB led */

/* I2S port and GPIOs */
#define I2S_BCK_IO      (GPIO_NUM_15)
#define I2S_WS_IO       (GPIO_NUM_16)
#define I2S_DI_IO       (GPIO_NUM_17)

/* H bridge GPIOs */
#define MOTOR_A_1_IO 10
#define MOTOR_A_2_IO 11
#define MOTOR_B_1_IO 12
#define MOTOR_B_2_IO 13

/* servo motor GPIOs */
#define SERVO_STRUM_GPIO 8 // GPIO number for strum servo
#ifdef CONFIG_HW_PROTOTYPE
#define SERVO_FRET_GPIO -1  // GPIO number for fret servo
#endif // CONFIG_HW_PROTOTYPE

/* stepper motor GPIOs */
#define STEP_MOTOR_GPIO_EN 4
#define STEP_MOTOR_GPIO_STEP 6
// #define STEP_MOTOR_GPIO_DIR 40
// #define STEP_MOTOR_MODE0_PIN 41
// #define STEP_MOTOR_MODE1_PIN 42
// #define STEP_MOTOR_MODE2_PIN 2
// TODO: temp
#define STEP_MOTOR_GPIO_DIR 21
#define STEP_MOTOR_MODE0_PIN 47
#define STEP_MOTOR_MODE1_PIN 48
#define STEP_MOTOR_MODE2_PIN 45

#endif // __GPIO_PIN_CONFIG_H__
