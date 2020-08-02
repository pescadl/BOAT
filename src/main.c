/*
 * @file main.c
 */

#include "../inc/boat.h"

/******************************************************************************
 * @brief main
 ******************************************************************************/
int main(void)
{
    Boat_Init();

    while(1)
    {
        Boat_Input();
        Boat_Process();
        Boat_Output();
    }
}
