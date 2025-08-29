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
#ifdef HW_V1_2_TEMP
#define MOTOR_B_1A_IO 10
#define MOTOR_B_1B_IO 11
#define MOTOR_A_1A_IO 12
#define MOTOR_A_1B_IO 13
#else
#define MOTOR_B_1B_IO 10
#define MOTOR_B_1A_IO 11
#define MOTOR_A_1B_IO 12
#define MOTOR_A_1A_IO 13
#endif

/* servo motor GPIOs */
#define SERVO_STRUM_GPIO 8 // GPIO number for strum servo
#ifdef CONFIG_HW_PROTOTYPE
#define SERVO_FRET_GPIO -1  // GPIO number for fret servo
#endif // CONFIG_HW_PROTOTYPE
#define SERVO_PWR_ADC 9

/* stepper motor GPIOs */
#ifdef HW_V1_2_TEMP
#define STEP_MOTOR_GPIO_EN 4
#define STEP_MOTOR_GPIO_STEP 6
#define STEP_MOTOR_GPIO_DIR 21
#define STEP_MOTOR_MODE0_PIN 47
#define STEP_MOTOR_MODE1_PIN 48
#define STEP_MOTOR_MODE2_PIN 45
#else
#define STEP_MOTOR_GPIO_EN 39
#define STEP_MOTOR_GPIO_DIR 1
#define STEP_MOTOR_GPIO_STEP 2
#define STEP_MOTOR_MODE0_PIN 40
#define STEP_MOTOR_MODE1_PIN 41
#define STEP_MOTOR_MODE2_PIN 42
#endif

// trigger GPIO for frequency measurement
#define FREQ_TRIGG_IO 0



#endif // __GPIO_PIN_CONFIG_H__
