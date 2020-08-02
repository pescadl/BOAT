/*
 * @file motor.h
 */

#ifndef MOTOR_H
#define MOTOR_H

/******************************************************************************
 * @brief Initializes Motor
 ******************************************************************************/
void Motor_Init(void);

/******************************************************************************
 * @brief Turns the motor to the left position
 ******************************************************************************/
void Motor_TurnLeft(void);

/******************************************************************************
 * @brief Turns the motor straight
 ******************************************************************************/
void Motor_TurnStraight(void);

/******************************************************************************
 * @brief Turns the motor to the right position
 ******************************************************************************/
void Motor_TurnRight(void);

#endif /* !MOTOR_H */
