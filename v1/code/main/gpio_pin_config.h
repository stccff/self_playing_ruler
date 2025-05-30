#ifndef __GPIO_PIN_CONFIG_H__
#define __GPIO_PIN_CONFIG_H__

#include <stdint.h>

/* I2S port and GPIOs */
#define I2S_BCK_IO      (GPIO_NUM_15)
#define I2S_WS_IO       (GPIO_NUM_16)
#define I2S_DI_IO       (GPIO_NUM_17)

/* electromagnet GPIOs */
#define MAGNET_TOP_A_GPIO 10
#define MAGNET_TOP_B_GPIO 11
#define MAGNET_BOTTOM_A_GPIO 12
#define MAGNET_BOTTOM_B_GPIO 13

/* servo motor GPIOs */
#define SERVO_STRUM_GPIO 19 // GPIO number for strum servo
#ifdef CONFIG_SERVO_FRET
#define SERVO_FRET_GPIO 20  // GPIO number for fret servo
#endif // CONFIG_SERVO_FRET

/* stepper motor GPIOs */
#define STEP_MOTOR_GPIO_EN 0
#define STEP_MOTOR_GPIO_DIR 2
#define STEP_MOTOR_GPIO_STEP 4
#define STEP_MOTOR_MODE0_PIN 40
#define STEP_MOTOR_MODE1_PIN 41
#define STEP_MOTOR_MODE2_PIN 42

#endif // __GPIO_PIN_CONFIG_H__
