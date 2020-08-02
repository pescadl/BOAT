#include "../inc/motor.h"
#include "../inc/hardware.h"

#define STATEMACHINE_SIZE 4

/******************************************************************************
 * PRIVATE FUNCTIONS
 ******************************************************************************/

static void motor1_on(void)
{
    GPIO_PinOutSet(MOTOR_1_PORT, MOTOR_1_PIN);
}

static void motor1_off(void)
{
    GPIO_PinOutClear(MOTOR_1_PORT, MOTOR_1_PIN);
}

static void motor2_on(void)
{
    GPIO_PinOutSet(MOTOR_2_PORT, MOTOR_2_PIN);
}

static void motor2_off(void)
{
    GPIO_PinOutClear(MOTOR_2_PORT, MOTOR_2_PIN);
}

static void motor3_on(void)
{
    GPIO_PinOutSet(MOTOR_3_PORT, MOTOR_3_PIN);
}

static void motor3_off(void)
{
    GPIO_PinOutClear(MOTOR_3_PORT, MOTOR_3_PIN);
}

static void motor4_on(void)
{
    GPIO_PinOutSet(MOTOR_4_PORT, MOTOR_4_PIN);
}

static void motor4_off(void)
{
    GPIO_PinOutClear(MOTOR_4_PORT, MOTOR_4_PIN);
}

void initGPIO(void)
{
    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(MOTOR_1_PORT, MOTOR_1_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(MOTOR_2_PORT, MOTOR_2_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(MOTOR_3_PORT, MOTOR_3_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(MOTOR_4_PORT, MOTOR_4_PIN, gpioModePushPull, 0);
}

static void halfstep1(void)
{
    motor1_on();
    motor2_off();
    motor3_off();
    motor4_off();
}

static void halfstep2(void)
{
    motor1_on();
    motor2_on();
    motor3_off();
    motor4_off();
}

static void halfstep3(void)
{
    motor1_off();
    motor2_on();
    motor3_off();
    motor4_off();
}

static void halfstep4(void)
{
    motor1_off();
    motor2_on();
    motor3_on();
    motor4_off();
}

static void halfstep5(void)
{
    motor1_off();
    motor2_off();
    motor3_on();
    motor4_off();
}

static void halfstep6(void)
{
    motor1_off();
    motor2_off();
    motor3_on();
    motor4_on();
}

static void halfstep7(void)
{
    motor1_off();
    motor2_off();
    motor3_off();
    motor4_on();
}

static void halfstep8(void)
{
    motor1_on();
    motor2_off();
    motor3_off();
    motor4_on();
}


static void (*statemachine[STATEMACHINE_SIZE]) (void) = {
    halfstep1,
    //halfstep2,
    halfstep3,
    //halfstep4,
    halfstep5,
    //halfstep6,
    halfstep7,
    //halfstep8
};

// TODO: Replace dumb delay with awesome TIMER
static void delay(void) 
{
    int i;
    while(i < 9000) 
    {
        i++;
    }
}

static int statemachine_index;

/******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************
 * @brief Initializes Motor
 ******************************************************************************/
void Motor_Init(void)
{
    statemachine_index = 0;
    initGPIO();
}

/******************************************************************************
 * @brief Turns the motor clockwise
 ******************************************************************************/
void Motor_TurnCW(void)
{
    statemachine[statemachine_index]();
    statemachine_index--;
    if(statemachine_index < 0) statemachine_index = STATEMACHINE_SIZE-1;
    delay();
}

/******************************************************************************
 * @brief Turns the motor counter-clockwise
 ******************************************************************************/
void Motor_TurnCCW(void)
{
    statemachine[statemachine_index]();
    statemachine_index = (statemachine_index + 1) % STATEMACHINE_SIZE;
    delay();
}
