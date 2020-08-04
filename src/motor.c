/******************************************************************************
 * @file motor.c
 ******************************************************************************/

#include "em_cmu.h"

#include "../inc/motor.h"
#include "../inc/hardware.h"



/******************************************************************************
 *                                   MACROS                                   *
 ******************************************************************************/

#define STATOR_SM_SIZE 4



/******************************************************************************
 *                                   ENUMS                                    *
 ******************************************************************************/

typedef enum MOTOR_TURN_STATE
{
    MOTOR_TURN_LEFT,
    MOTOR_TURN_STRAIGHT,
    MOTOR_TURN_RIGHT
} motor_turn_state_t;



/******************************************************************************
 *                              PRIVATE VARIABLES                             *
 ******************************************************************************/
   
static motor_turn_state_t motor_turn_state;

static void halfstep1(void);
static void halfstep3(void);
static void halfstep5(void);
static void halfstep7(void);

static void (*stator_sm[STATOR_SM_SIZE]) (void) = {
    halfstep1,
    //halfstep2,
    halfstep3,
    //halfstep4,
    halfstep5,
    //halfstep6,
    halfstep7,
    //halfstep8
};
static int stator_sm_index;



/******************************************************************************
 *                              PRIVATE FUNCTIONS                             *
 ******************************************************************************/

static void stator1On(void)
{
    GPIO_PinOutSet(MOTOR_STATOR_1_PORT, MOTOR_STATOR_1_PIN);
}

static void stator1Off(void)
{
    GPIO_PinOutClear(MOTOR_STATOR_1_PORT, MOTOR_STATOR_1_PIN);
}

static void stator2On(void)
{
    GPIO_PinOutSet(MOTOR_STATOR_2_PORT, MOTOR_STATOR_2_PIN);
}

static void stator2Off(void)
{
    GPIO_PinOutClear(MOTOR_STATOR_2_PORT, MOTOR_STATOR_2_PIN);
}

static void stator3On(void)
{
    GPIO_PinOutSet(MOTOR_STATOR_3_PORT, MOTOR_STATOR_3_PIN);
}

static void stator3Off(void)
{
    GPIO_PinOutClear(MOTOR_STATOR_3_PORT, MOTOR_STATOR_3_PIN);
}

static void stator4On(void)
{
    GPIO_PinOutSet(MOTOR_STATOR_4_PORT, MOTOR_STATOR_4_PIN);
}

static void stator4Off(void)
{
    GPIO_PinOutClear(MOTOR_STATOR_4_PORT, MOTOR_STATOR_4_PIN);
}

static void initGPIO(void)
{
    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(MOTOR_STATOR_1_PORT, MOTOR_STATOR_1_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(MOTOR_STATOR_2_PORT, MOTOR_STATOR_2_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(MOTOR_STATOR_3_PORT, MOTOR_STATOR_3_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(MOTOR_STATOR_4_PORT, MOTOR_STATOR_4_PIN, gpioModePushPull, 0);
}

static void halfstep1(void)
{
    stator1On();
    stator2Off();
    stator3Off();
    stator4Off();
}

static void halfstep2(void)
{
    stator1On();
    stator2On();
    stator3Off();
    stator4Off();
}

static void halfstep3(void)
{
    stator1Off();
    stator2On();
    stator3Off();
    stator4Off();
}

static void halfstep4(void)
{
    stator1Off();
    stator2On();
    stator3On();
    stator4Off();
}

static void halfstep5(void)
{
    stator1Off();
    stator2Off();
    stator3On();
    stator4Off();
}

static void halfstep6(void)
{
    stator1Off();
    stator2Off();
    stator3On();
    stator4On();
}

static void halfstep7(void)
{
    stator1Off();
    stator2Off();
    stator3Off();
    stator4On();
}

static void halfstep8(void)
{
    stator1On();
    stator2Off();
    stator3Off();
    stator4On();
}

// TODO: Replace dumb delay with awesome TIMER
static void delay(void) 
{
    int i;
    while(i < 9000) 
    {
        i++;
    }
}

/******************************************************************************
 * @brief Turns the motor clockwise by one full step
 ******************************************************************************/
static void stepCW(void)
{
    stator_sm[stator_sm_index]();
    stator_sm_index--;
    if(stator_sm_index < 0)
    {
        stator_sm_index = STATOR_SM_SIZE-1;
    }
    delay();
}

/******************************************************************************
 * @brief Turns the motor counter-clockwise by one full step
 ******************************************************************************/
static void stepCCW(void)
{
    stator_sm[stator_sm_index]();
    stator_sm_index = (stator_sm_index + 1) % STATOR_SM_SIZE;
    delay();
}

/******************************************************************************
 * @brief Turns the motor to a specific angle relative to what it's already at
 * @param angle The angle to turn to. Ranges from -180 to 180
 ******************************************************************************/
static void motorTurnAngle(int angle)
{
    int i;
    int steps = ((2038 * 1e6) / 360 * angle) / 1e6;

    if (angle >= 0)
    {
        for (i = 0; i < steps; i++)
        {
            stepCW();
        }
    }
    else
    {
        steps *= -1;
        for (i = 0; i < steps; i++)
        {
            stepCCW();
        }
    }
}



/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/******************************************************************************
 * @brief Initializes Motor
 ******************************************************************************/
void Motor_Init(void)
{
    motor_turn_state = MOTOR_TURN_STRAIGHT;
    stator_sm_index = 0;

    initGPIO();
}

/******************************************************************************
 * @brief Turns the motor to the left
 ******************************************************************************/
void Motor_TurnLeft()
{
    if (motor_turn_state == MOTOR_TURN_STRAIGHT)
    {
        motorTurnAngle(-45);
    }
    else if (motor_turn_state == MOTOR_TURN_RIGHT)
    {
        motorTurnAngle(-90);
    }

    motor_turn_state = MOTOR_TURN_LEFT;
}

/******************************************************************************
 * @brief Turns the motor straight
 ******************************************************************************/
void Motor_TurnStraight()
{
    if (motor_turn_state == MOTOR_TURN_LEFT)
    {
        motorTurnAngle(45);
    }
    else if (motor_turn_state == MOTOR_TURN_RIGHT)
    {
        motorTurnAngle(-45);
    }

    motor_turn_state = MOTOR_TURN_STRAIGHT;
}

/******************************************************************************
 * @brief Turns the motor to the right
 ******************************************************************************/
void Motor_TurnRight()
{
    if (motor_turn_state == MOTOR_TURN_LEFT)
    {
        motorTurnAngle(90);
    }
    else if (motor_turn_state == MOTOR_TURN_STRAIGHT)
    {
        motorTurnAngle(45);
    }

    motor_turn_state = MOTOR_TURN_RIGHT;
}
