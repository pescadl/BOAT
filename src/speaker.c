/******************************************************************************
 * @file speaker.c
 ******************************************************************************/

#include "em_cmu.h"
#include "em_vdac.h"
#include "em_ldma.h"

#include "../inc/speaker.h"
#include "../inc/hardware.h"

// 32 point sine table
#define SINE_TABLE_SIZE 32
static const uint16_t sineTable[SINE_TABLE_SIZE] = {
  2048 , 2447 , 2831 , 3185 , 3495 , 3750 , 3939 , 4056 ,
  4095 , 4056 , 3939 , 3750 , 3495 , 3185 , 2831 , 2447 ,
  2048 , 1648 , 1264 , 910  , 600  , 345  , 156  , 39   ,
  0    , 39   , 156  , 345  , 600  , 910  , 1264 , 1648 ,
};

/******************************************************************************
 *                              PRIVATE FUNCTIONS                             *
 ******************************************************************************/

void VDAC0_IRQHandler(void)
{
  int flags = VDAC0->IF;
  VDAC_IntClear(VDAC0, flags);
  if (flags & VDAC_IF_CH0CD)
  {
    GPIO_PinOutToggle(gpioPortD, 0);
  }
  else if (flags & VDAC_IF_CH0BL)
  {
    GPIO_PinOutToggle(gpioPortD, 1);
  }
  else if ((flags & VDAC_IF_CH0UF) || (flags & VDAC_IF_CH0OF))
//  else if (flags & VDAC_IF_CH0UF)
  {
//    GPIO_PinOutToggle(gpioPortD, 1);
      GPIO_PinOutSet(gpioPortD, 1);
  }
}

/******************************************************************************
 * @brief
 *    VDAC initialization
 *****************************************************************************/
static void initVdac(void)
{
  // Enable the VDAC clock
  CMU_ClockEnable(cmuClock_VDAC0, true);

  // Initialize the VDAC
  VDAC_Init_TypeDef init = VDAC_INIT_DEFAULT;
  init.prescaler = VDAC_PrescaleCalc(96000*8, !init.asyncClockMode, 0);
  init.reference = vdacRefAvdd;
  init.refresh = vdacRefresh8;
  init.ch0ResetPre = true;
  VDAC_Init(VDAC0, &init);

  // Initialize VDAC channel 0
  VDAC_InitChannel_TypeDef initChannel0 = VDAC_INITCHANNEL_DEFAULT;
  // Trigger conversions off of the refresh timer
  initChannel0.trigMode = vdacTrigModeRefresh;
  VDAC_InitChannel(VDAC0, &initChannel0, 0);

  // Set the settle time to zero for maximum update rate (mask it out)
  VDAC0->OPA[0].TIMER &= ~(_VDAC_OPA_TIMER_SETTLETIME_MASK);

  // TODO: remove
  // Enable GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);
  // Configure as output
  GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);
  NVIC_EnableIRQ(VDAC0_IRQn);
  VDAC_IntEnable(VDAC0, VDAC_IEN_CH0CD);
//  VDAC_IntEnable(VDAC0, VDAC_IEN_CH0BL);
  VDAC_IntEnable(VDAC0, VDAC_IEN_CH0OF);
  VDAC_IntEnable(VDAC0, VDAC_IEN_CH0UF);

  // Enable VDAC channel 0
  VDAC_Enable(VDAC0, 0, true);
}

/**************************************************************************//**
 * @brief
 *    Initialize the LDMA module
 *
 * @note
 *    The descriptor object needs to at least have static scope persistence so
 *    that the reference to the object is valid beyond its first use in
 *    initialization. This is because this code loops back to the same
 *    descriptor after every dma transfer. If the reference isn't valid anymore,
 *    then all dma transfers after the first one will fail.
 ******************************************************************************/
static void initLdma(void)
{
  // Descriptor loops through the sine table and outputs its values to the VDAC
  static LDMA_Descriptor_t loopDescriptor =
    LDMA_DESCRIPTOR_LINKREL_M2P_BYTE(&sineTable[0],
                                     &VDAC0->CH0DATA,
                                     SINE_TABLE_SIZE,
                                     0);
  // Don't trigger interrupt when transfer is done
  loopDescriptor.xfer.doneIfs = 0;
  // Transfer halfwords (VDAC data register is 12 bits)
  loopDescriptor.xfer.size = ldmaCtrlSizeHalf;

  // Transfer configuration and trigger selection
  // Trigger when VDAC0_CH0DATA is empty
  LDMA_TransferCfg_t transferConfig =
    LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_VDAC0_CH0);

  // LDMA initialization
  LDMA_Init_t init = LDMA_INIT_DEFAULT;
  LDMA_Init(&init);

  // Start the transfer
  uint32_t channelNum = 0;
  LDMA_StartTransfer(channelNum, &transferConfig, &loopDescriptor);
}

/******************************************************************************
 * @brief
 *    Need to disable microphone to make the mux select the DAC
 *****************************************************************************/
static void disableMicrophone(void)
{
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Set PA8 low to disable microphone
  GPIO_PinModeSet(gpioPortA, 8, gpioModePushPull, 0);
}



/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/******************************************************************************
 * @brief Initializes Motor
 ******************************************************************************/
void Speaker_Init(void)
{
  disableMicrophone();
  initVdac();
  initLdma();
}
