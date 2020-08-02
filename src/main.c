#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

void initGPIO(void)
{
    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(gpioPortC, 5, gpioModePushPull, 0);
    GPIO_PinModeSet(gpioPortC, 4, gpioModePushPull, 0);
    GPIO_PinModeSet(gpioPortE, 9, gpioModePushPull, 0);
    GPIO_PinModeSet(gpioPortE, 8, gpioModePushPull, 0);
}

void red_on(void)
{
    GPIO_PinOutSet(gpioPortC, 5);
}

void red_off(void)
{
    GPIO_PinOutClear(gpioPortC, 5);
}

void blue_on(void) 
{
    GPIO_PinOutSet(gpioPortC, 4);
}

void blue_off(void) 
{
    GPIO_PinOutClear(gpioPortC, 4);
}

void white_on(void) 
{
    GPIO_PinOutSet(gpioPortE,  9);
}

void white_off(void) 
{
    GPIO_PinOutClear(gpioPortE,  9);
}

void yellow_on(void) 
{
    GPIO_PinOutSet(gpioPortE,  8);
}

void yellow_off(void) 
{
    GPIO_PinOutClear(gpioPortE,  8);
}

void delay(void) 
{
    int i;
    while(i <9000) 
    {
        i++;
    }
}

int main(void)
{
    int i = 0;
    int m = 3;

    // Chip errata
    CHIP_Init();

    // Initializations
    initGPIO();

    while(1)
    {
        if(i%m == 0)
        {
            red_on();
            blue_off();
            white_off();
            yellow_off();
            delay();
        }
        i++;

        if(i%m == 0) 
        {
            red_on();
            blue_on();
            white_off();
            yellow_off();
            delay();
        }
        i++;

        if(i%m == 0) 
        {
            red_off();
            blue_on();
            white_off();
            yellow_off();
            delay();
        }
        i++;

        if(i%m == 0) 
        {
            red_off();
            blue_on();
            white_on();
            yellow_off();
            delay();
        }
        i++;

        if(i%m == 0) 
        {
            red_off();
            blue_off();
            white_on();
            yellow_off();
            delay();
        }
        i++;

        if(i%m == 0) 
        {
            red_off();
            blue_off();
            white_on();
            yellow_on();
            delay();
        }
        i++;

        if(i%m == 0) 
        {
            red_off();
            blue_off();
            white_off();
            yellow_on();
            delay();
        }
        i++;

        if(i%m == 0) 
        {
            red_on();
            blue_off();
            white_off();
            yellow_on();
            delay();
        }
        i++;
    }
}
