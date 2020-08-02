/******************************************************************************
 * @file lights.c
 ******************************************************************************/

#include "../inc/lights.h"
#include "../inc/ldr.h"
#include "../inc/hardware.h"

#include "em_cmu.h"
#include "em_gpio.h"



/******************************************************************************
 *                              PRIVATE FUNCTIONS                             *
 ******************************************************************************/

static void headLightsOn(void)
{
    GPIO_PinOutClear(LIGHTS_HEAD_LEFT_PORT, LIGHTS_HEAD_LEFT_PIN);
    GPIO_PinOutClear(LIGHTS_HEAD_RIGHT_PORT, LIGHTS_HEAD_RIGHT_PIN);
}

static void headLightsOff(void)
{
    GPIO_PinOutSet(LIGHTS_HEAD_LEFT_PORT, LIGHTS_HEAD_LEFT_PIN);
    GPIO_PinOutSet(LIGHTS_HEAD_RIGHT_PORT, LIGHTS_HEAD_RIGHT_PIN);
}

static void brakeLightsOn(void)
{
    GPIO_PinOutClear(LIGHTS_BRAKE_LEFT_PORT, LIGHTS_BRAKE_LEFT_PIN);
    GPIO_PinOutClear(LIGHTS_BRAKE_RIGHT_PORT, LIGHTS_BRAKE_RIGHT_PIN);
}

static void brakeLightsOff(void)
{
    GPIO_PinOutSet(LIGHTS_BRAKE_LEFT_PORT, LIGHTS_BRAKE_LEFT_PIN);
    GPIO_PinOutSet(LIGHTS_BRAKE_RIGHT_PORT, LIGHTS_BRAKE_RIGHT_PIN);
}

static void turnLightLeftToggle(void)
{
    GPIO_PinOutToggle(LIGHTS_TURN_LEFT_PORT, LIGHTS_TURN_LEFT_PIN);
}

static void turnLightRightToggle(void)
{
    GPIO_PinOutToggle(LIGHTS_TURN_RIGHT_PORT, LIGHTS_TURN_RIGHT_PIN);
}

static void turnLightsOn(void)
{
    GPIO_PinOutClear(LIGHTS_TURN_LEFT_PORT, LIGHTS_TURN_LEFT_PIN);
    GPIO_PinOutClear(LIGHTS_TURN_RIGHT_PORT, LIGHTS_TURN_RIGHT_PIN);
}

static void turnLightsOff(void)
{
    GPIO_PinOutSet(LIGHTS_TURN_LEFT_PORT, LIGHTS_TURN_LEFT_PIN);
    GPIO_PinOutSet(LIGHTS_TURN_RIGHT_PORT, LIGHTS_TURN_RIGHT_PIN);
}



/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

void Lights_Init(void)
{
    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(LIGHTS_HEAD_LEFT_PORT,  LIGHTS_HEAD_LEFT_PIN,  gpioModePushPull, 1);
    GPIO_PinModeSet(LIGHTS_HEAD_RIGHT_PORT, LIGHTS_HEAD_RIGHT_PIN, gpioModePushPull, 1);

    GPIO_PinModeSet(LIGHTS_BRAKE_LEFT_PORT,  LIGHTS_BRAKE_LEFT_PIN,  gpioModePushPull, 1);
    GPIO_PinModeSet(LIGHTS_BRAKE_RIGHT_PORT, LIGHTS_BRAKE_RIGHT_PIN, gpioModePushPull, 1);

    GPIO_PinModeSet(LIGHTS_TURN_LEFT_PORT,  LIGHTS_TURN_LEFT_PIN,  gpioModePushPull, 1);
    GPIO_PinModeSet(LIGHTS_TURN_RIGHT_PORT, LIGHTS_TURN_RIGHT_PIN, gpioModePushPull, 1);
}

void Lights_Update(ambience_t ambience)
{
    if(ambience == AMBIENCE_BRIGHT)
    {
        headLightsOff();
    }
    else if(ambience == AMBIENCE_DARK)
    {
        headLightsOn();
    }
}
