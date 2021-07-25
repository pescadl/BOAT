/******************************************************************************
 * @file stepper_motor.c
 * @brief driver code for a unipolar 4-phase stepper motor (28BYJ-48)
 ******************************************************************************/

#include "em_cmu.h"
#include "em_timer.h"

#include "../inc/stepper_motor.h"
#include "../inc/hardware.h"



/******************************************************************************
 *                                   MACROS                                   *
 ******************************************************************************/

// The number of full steps required to rotate the motor by 360 degrees
#define FULL_ROTATION_STEPS 2048

// The motor frequency in Hz
#define MOTOR_FREQ 200

// The number of coils/phases
#define NUM_COILS 4

// Clockwise or counterclockwise direction
#define CCW 0
#define CW 1


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

// Motor port connections
GPIO_Port_TypeDef coilPorts[NUM_COILS] =
{
    STEPPER_MOTOR_COIL_1_PORT,
    STEPPER_MOTOR_COIL_2_PORT,
    STEPPER_MOTOR_COIL_3_PORT,
    STEPPER_MOTOR_COIL_4_PORT
};

// Motor pin connections
uint8_t coilPins[NUM_COILS] =
{
    STEPPER_MOTOR_COIL_1_PIN,
    STEPPER_MOTOR_COIL_2_PIN,
    STEPPER_MOTOR_COIL_3_PIN,
    STEPPER_MOTOR_COIL_4_PIN
};

int direction;
int num_steps;
int current_step;



/******************************************************************************
 *                              PRIVATE FUNCTIONS                             *
 ******************************************************************************/

/******************************************************************************
 * @brief Magnetize the coil
 ******************************************************************************/
void coilOn(GPIO_Port_TypeDef gpioPort, int pin)
{
    GPIO_PinOutSet(gpioPort, pin);
}

/******************************************************************************
 * @brief Demagnetize the coil
 ******************************************************************************/
void coilOff(GPIO_Port_TypeDef gpioPort, int pin)
{
    GPIO_PinOutClear(gpioPort, pin);
}

/******************************************************************************
 * @brief Turns on the specified coil, and turns off the remaining coils
 ******************************************************************************/
void coilOutput(int coil)
{
    int i;

    for(i=0; i<coil; i++)
    {
        coilOff(coilPorts[i], coilPins[i]);
    }

    coilOn(coilPorts[coil], coilPins[coil]);

    for(i=coil+1; i<NUM_COILS; i++)
    {
        coilOff(coilPorts[i], coilPins[i]);
    }
}

/******************************************************************************
 * @brief Init GPIO
 ******************************************************************************/
static void initGPIO(void)
{
    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(coilPorts[0], coilPins[0], gpioModePushPull, 0);
    GPIO_PinModeSet(coilPorts[1], coilPins[1], gpioModePushPull, 0);
    GPIO_PinModeSet(coilPorts[2], coilPins[2], gpioModePushPull, 0);
    GPIO_PinModeSet(coilPorts[3], coilPins[3], gpioModePushPull, 0);
}

/******************************************************************************
 * @brief Init Timer
 ******************************************************************************/
void initTimer(void)
{
    // Enable clock for TIMER1 module
    CMU_ClockEnable(cmuClock_TIMER1, true);

    // Configure TIMER1 Compare/Capture for output compare
    TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
    timerCCInit.mode = timerCCModeCompare;
    TIMER_InitCC(TIMER1, 0, &timerCCInit);

    // Initialize TIMER1 with the highest prescaler
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.enable = false;
    timerInit.prescale = timerPrescale1024;
    TIMER_Init(TIMER1, &timerInit);

    // Set the TIMER1 overflow at MOTOR_FREQ Hz
    uint32_t topValue = CMU_ClockFreqGet(cmuClock_HFPER) /
                        (2*MOTOR_FREQ * (1 << timerPrescale1024)) - 1;
    TIMER_TopSet(TIMER1, topValue);

    // Enable TIMER1 interrupts
    TIMER_IntEnable(TIMER1, TIMER_IEN_OF);
    NVIC_EnableIRQ(TIMER1_IRQn);
}

/******************************************************************************
 * @brief Handle Timer overflow event and increment step
 ******************************************************************************/
void TIMER1_IRQHandler(void)
{
    // Acknowledge the interrupt
    uint32_t flags = TIMER_IntGet(TIMER1);
    TIMER_IntClear(TIMER1, flags);

    // Rotate the motor by one full step
    current_step++;

    if(direction == CW)
    {
        coilOutput(current_step % NUM_COILS);
    }
    else
    {
        coilOutput((num_steps - current_step) % NUM_COILS);
    }

    // Stop rotating the motor if the desired angle is reached
    if(current_step == num_steps)
    {
      TIMER_Enable(TIMER1, false);
      current_step = 0;
    }
}

/******************************************************************************
 * @brief Returns the number of steps required to rotate a specified angle
 ******************************************************************************/
int calculateSteps(int angle)
{
    return (angle * FULL_ROTATION_STEPS) / 360;
}



/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/******************************************************************************
 * @brief Initializes stepper motor
 ******************************************************************************/
void Stepper_Motor_Init(void)
{
    motor_turn_state = MOTOR_TURN_STRAIGHT;

    current_step = 0;

    initGPIO();
    initTimer();
}

/******************************************************************************
 * @brief Turns the motor to the left
 ******************************************************************************/
void Stepper_Motor_TurnLeft(void)
{
    if (motor_turn_state == MOTOR_TURN_STRAIGHT)
    {
        num_steps = calculateSteps(45);
        direction = CW;
    }
    else if (motor_turn_state == MOTOR_TURN_RIGHT)
    {
        num_steps = calculateSteps(90);
        direction = CW;
    }

    motor_turn_state = MOTOR_TURN_LEFT;
    TIMER_Enable(TIMER1, true);
}

/******************************************************************************
 * @brief Turns the motor straight
 ******************************************************************************/
void Stepper_Motor_TurnStraight(void)
{
    if (motor_turn_state == MOTOR_TURN_LEFT)
    {
        num_steps = calculateSteps(45);
        direction = CCW;
    }
    else if (motor_turn_state == MOTOR_TURN_RIGHT)
    {
        num_steps = calculateSteps(45);
        direction = CW;
    }

    motor_turn_state = MOTOR_TURN_STRAIGHT;
    TIMER_Enable(TIMER1, true);
}

/******************************************************************************
 * @brief Turns the motor to the right
 ******************************************************************************/
void Stepper_Motor_TurnRight(void)
{
    if (motor_turn_state == MOTOR_TURN_LEFT)
    {
        num_steps = calculateSteps(90);
        direction = CCW;
    }
    else if (motor_turn_state == MOTOR_TURN_STRAIGHT)
    {
        num_steps = calculateSteps(45);
        direction = CCW;
    }

    motor_turn_state = MOTOR_TURN_RIGHT;
    TIMER_Enable(TIMER1, true);
}
