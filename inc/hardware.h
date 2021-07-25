/******************************************************************************
 * @file hardware.h
 ******************************************************************************/

#ifndef HARDWARE_H
#define HARDWARE_H

#include "em_gpio.h"



/******************************************************************************
 *                                   MACROS                                   *
 ******************************************************************************/

#define LDR_APORT                   acmpInputAPORT1XCH0
#define LDR_PORT                    gpioPortA
#define LDR_PIN                     0

#define STEPPER_MOTOR_COIL_1_PORT   gpioPortA
#define STEPPER_MOTOR_COIL_1_PIN    1 

#define STEPPER_MOTOR_COIL_2_PORT   gpioPortA
#define STEPPER_MOTOR_COIL_2_PIN    2 

#define STEPPER_MOTOR_COIL_3_PORT   gpioPortA
#define STEPPER_MOTOR_COIL_3_PIN    3 

#define STEPPER_MOTOR_COIL_4_PORT   gpioPortC
#define STEPPER_MOTOR_COIL_4_PIN    4

#define LIGHTS_HEAD_LEFT_PORT       gpioPortC
#define LIGHTS_HEAD_LEFT_PIN        5

#define LIGHTS_HEAD_RIGHT_PORT      gpioPortE
#define LIGHTS_HEAD_RIGHT_PIN       4

#define LIGHTS_BRAKE_LEFT_PORT      gpioPortE
#define LIGHTS_BRAKE_LEFT_PIN       8

#define LIGHTS_BRAKE_RIGHT_PORT     gpioPortE
#define LIGHTS_BRAKE_RIGHT_PIN      9

#define LIGHTS_TURN_LEFT_PORT       gpioPortE
#define LIGHTS_TURN_LEFT_PIN        10

#define LIGHTS_TURN_RIGHT_PORT      gpioPortE
#define LIGHTS_TURN_RIGHT_PIN       11



#endif /* !HARDWARE_H */
