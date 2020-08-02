/******************************************************************************
 * @file boat.c
 ******************************************************************************/

#include "em_device.h"
#include "em_chip.h"

#include "../inc/boat.h"
#include "../inc/motor.h"
#include "../inc/ldr.h"
#include "../inc/lights.h"



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
    Lights_Init();
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

    Lights_Update(LDR_getAmbience());
}
