/******************************************************************************
 * @file ldr.c
 ******************************************************************************/

#include "../inc/ldr.h"
#include "../inc/hardware.h"

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_acmp.h"



/******************************************************************************
 *                              PRIVATE FUNCTIONS                             *
 ******************************************************************************/

/******************************************************************************
 * @brief Initializes LDR's GPIO
 ******************************************************************************/
void initGPIO(void)
{
    CMU_ClockEnable(cmuClock_GPIO, true);
    
    GPIO_PinModeSet(LDR_PORT, LDR_PIN, gpioModeInput, 0);
}

/******************************************************************************
 * @brief Initializes LDR's ACMP
 ******************************************************************************/
void initACMP(void)
{
    CMU_ClockEnable(cmuClock_ACMP0, true);

    ACMP_Init_TypeDef acmp0_init = {
        false,                    // fullBias
        0x7,                      // biasProg
        false,                    // No interrupt on falling edge
        false,                    // No interrupt on rising edge
        acmpInputRangeFull,       // Input range from 0 to VDD
        acmpAccuracyLow,          // Low accuracy, less current usage
        acmpPowerSourceAvdd,      // Use the AVDD supply
        acmpHysteresisLevel5,     // Use hysteresis level 5 when output is 0
        acmpHysteresisLevel5,     // Use hysteresis level 5 when output is 1
        acmpVLPInputVADIV,        // Use VADIV as the VLP input source
        false,                    // Output 0 when ACMP is inactive
        false                     // Don't enable after init
    };

    // Set VB to default configuration of 1.25V
    ACMP_VBConfig_TypeDef vb_config = ACMP_VBCONFIG_DEFAULT;
    
    // Init and set ACMP channel
    ACMP_Init(ACMP0, &acmp0_init);
    
    // Configure the GPIO pins such that if PB9 is high, the output is logic high
    ACMP_ChannelSet(ACMP0, acmpInputVBDIV, LDR_APORT);
    
    ACMP_VBSetup(ACMP0, &vb_config);
    
    ACMP_Enable(ACMP0);
    
    // Wait for warmup
    while (!(ACMP0->STATUS & _ACMP_STATUS_ACMPACT_MASK)) ;
}



/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/******************************************************************************
 * @brief Initializes LDR
 ******************************************************************************/
void LDR_Init(void)
{
    initGPIO();
    initACMP();
}

/******************************************************************************
 * @brief Returns the ambient light based on LDR
 * @return Ambient light
 ******************************************************************************/
ambience_t LDR_getAmbience(void)
{
    if(ACMP0->STATUS & _ACMP_STATUS_ACMPOUT_MASK)
    {
        return AMBIENCE_BRIGHT;
    }
    else
    {
        return AMBIENCE_DARK;
    }
}
