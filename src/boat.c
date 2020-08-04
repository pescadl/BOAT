/******************************************************************************
 * @file boat.c
 ******************************************************************************/

#include "em_device.h"
#include "em_chip.h"

#include "../inc/boat.h"
#include "../inc/motor.h"
#include "../inc/ldr.h"



/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/******************************************************************************
 * @brief Init Boat
 ******************************************************************************/
void Boat_Init(void)
{
    CHIP_Init();
    Motor_Init();
    LDR_Init();
}

/******************************************************************************
 * @brief Gather Boat's input data
 ******************************************************************************/
void Boat_Input(void)
{
    // TODO    
}

/******************************************************************************
 * @brief Process Boat's input data
 ******************************************************************************/
void Boat_Process(void)
{
    // TODO
}

/******************************************************************************
 * @brief Output to Boat's peripherals
 ******************************************************************************/
static void delay(void) 
{
    int i;
    while(i < 900000) 
    {
        i++;
    }
}

void Boat_Output(void)
{
    // motor test
    // Motor_TurnRight();
    // delay();
    // Motor_TurnStraight();
    // delay();
    // Motor_TurnLeft();
    // delay();



    // ldr test
    if(LDR_getAmbientLight() == AMBIENCE_BRIGHT)
    {
        GPIO_PinOutSet(gpioPortA, 12);
    }
    else if(LDR_getAmbientLight() == AMBIENCE_DARK)
    {
        GPIO_PinOutClear(gpioPortA, 12);
    }
}
