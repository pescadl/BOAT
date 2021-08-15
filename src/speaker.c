/******************************************************************************
 * @file speaker.c
 ******************************************************************************/

#include "em_cmu.h"
#include "em_vdac.h"
#include "em_ldma.h"

#include "../inc/speaker.h"
#include "../inc/hardware.h"

/******************************************************************************
 *                                   MACROS                                   *
 ******************************************************************************/

// We want the maximum XFERCNT in decimal (e.g. 2^11 = 2048). We can achieve
// this using some fancy maths
#define MAX_XFER_COUNT ((_LDMA_CH_CTRL_XFERCNT_MASK >> _LDMA_CH_CTRL_XFERCNT_SHIFT) + 1)

#define NUM_LDMA_DESCRIPTORS (NUM_LDMA_DESCRIPTORS_A + 2)
#define NUM_LDMA_DESCRIPTORS_A (((int)(HONK_WAVEFORM_SIZE / MAX_XFER_COUNT))+1)
#define NUM_LDMA_DESCRIPTORS_A_LOOPS 20
#define LDMA_DESCRIPTOR_B_INDEX (NUM_LDMA_DESCRIPTORS - 2)
#define LDMA_DESCRIPTOR_C_INDEX (NUM_LDMA_DESCRIPTORS - 1)


/******************************************************************************
 *                              PRIVATE FUNCTIONS                             *
 ******************************************************************************/

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
  // Descriptor sequence is:
  // Loop(A0, A1, ..., An, NUM_LDMA_DESCRIPTORS_A_LOOPS), B, C
  //  - A0 through An are for playing the honk sine wave
  //  - We loop that for the desired loop count NUM_LDMA_DESCRIPTORS_A_LOOPS
  //  - Then descriptor B will deactivate the VDAC
  //  - Then descriptor C will deactivate the LDMA

  // Create the array of linked descriptors
  static LDMA_Descriptor_t ldmaDescriptors[NUM_LDMA_DESCRIPTORS];

  // The max transfer size is not enough to play the entire sine table so
  // we'll link the A-type descriptors together
  for (int i = 0; i < NUM_LDMA_DESCRIPTORS_A; i++)
  {
    if (i < NUM_LDMA_DESCRIPTORS_A-1)
    {
      // Each descriptor links to the next one
      ldmaDescriptors[i] = (LDMA_Descriptor_t)
        LDMA_DESCRIPTOR_LINKREL_M2P_BYTE(&HONK_WAVEFORM[i*MAX_XFER_COUNT],
                                         &VDAC0->CH0DATA,
                                         MAX_XFER_COUNT,
                                         1);
    }
    else
    {
      // Except the last descriptor, which links back to the first descriptor
      const uint16_t remainingTransfers =
        HONK_WAVEFORM_SIZE - (i*MAX_XFER_COUNT);
      ldmaDescriptors[i] = (LDMA_Descriptor_t)
        LDMA_DESCRIPTOR_LINKREL_M2P_BYTE(&HONK_WAVEFORM[i*MAX_XFER_COUNT],
                                         &VDAC0->CH0DATA,
                                         remainingTransfers,
                                         -(NUM_LDMA_DESCRIPTORS_A-1));
      // The last A-type descriptor will decrease the loop count
      ldmaDescriptors[i].xfer.decLoopCnt = 1;
    }
    // Don't trigger interrupt when transfer is done
    ldmaDescriptors[i].xfer.doneIfs = 0;
    // Transfer halfwords (VDAC data register is 12 bits)
    ldmaDescriptors[i].xfer.size = ldmaCtrlSizeHalf;
  }

  // Set up descriptor B for deactivating the VDAC and link to the next
  // descriptor
  ldmaDescriptors[LDMA_DESCRIPTOR_B_INDEX] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_SINGLE_WRITE(VDAC_CMD_CH0DIS,
                                 &VDAC0->CMD);
  ldmaDescriptors[LDMA_DESCRIPTOR_B_INDEX].wri.linkMode = ldmaLinkModeRel;
  ldmaDescriptors[LDMA_DESCRIPTOR_B_INDEX].wri.link = 1;
  ldmaDescriptors[LDMA_DESCRIPTOR_B_INDEX].wri.linkAddr = 1*4;

  // Set up descriptor C for deactivating the LDMA
  // TODO: ^
//  ldmaDescriptors[LDMA_DESCRIPTOR_C_INDEX] = (LDMA_Descriptor_t)
//      LDMA_DESCRIPTOR_SINGLE_WRITE(VDAC_CMD_CH0DIS,
//                                   &VDAC0->CMD);

  // Trigger when VDAC0_CH0DATA is empty
  LDMA_TransferCfg_t transferConfig = LDMA_TRANSFER_CFG_PERIPHERAL_LOOP(
    ldmaPeripheralSignal_VDAC0_CH0, NUM_LDMA_DESCRIPTORS_A_LOOPS);

  // LDMA initialization
  LDMA_Init_t init = LDMA_INIT_DEFAULT;
  LDMA_Init(&init);

  // Start the transfer
  uint32_t channelNum = 0;
  LDMA_StartTransfer(channelNum, &transferConfig, &ldmaDescriptors[0]);
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
