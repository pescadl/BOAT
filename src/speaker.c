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

// Number of iterations = (number of loops + 1)
#define NUM_LDMA_DESCRIPTORS_B_LOOPS 0

// Calculate the total number of LDMA descriptors
#define NUM_LDMA_DESCRIPTORS (NUM_LDMA_DESCRIPTORS_B + 4)
#define NUM_LDMA_DESCRIPTORS_B (((int)(HONK_WAVEFORM_SIZE / MAX_XFER_COUNT))+1)

// Determine descriptor order
#define LDMA_DESCRIPTOR_A_INDEX (0)
#define LDMA_DESCRIPTOR_B_INDEX (LDMA_DESCRIPTOR_A_INDEX + 1)
#define LDMA_DESCRIPTOR_C_INDEX (LDMA_DESCRIPTOR_B_INDEX + NUM_LDMA_DESCRIPTORS_B)
#define LDMA_DESCRIPTOR_D_INDEX (LDMA_DESCRIPTOR_C_INDEX + 1)
#define LDMA_DESCRIPTOR_E_INDEX (LDMA_DESCRIPTOR_D_INDEX + 1)


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
  init.prescaler = VDAC_PrescaleCalc(48000*8, !init.asyncClockMode, 0);
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
  VDAC0->CH0DATA = 0;
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
  // A, Loop(B0, B1, ..., Bn, NUM_LDMA_DESCRIPTORS_B_LOOPS), C, D, E
  //  - A: First ramp up to VDD/2 to avoid a popping sound from the speaker
  //  - B: B0 through Bn are for playing the honk sine wave
  //  -    We loop that for the desired loop count NUM_LDMA_DESCRIPTORS_B_LOOPS
  //  - C: Then ramp down to 0 to avoid a popping sound from the speaker
  //  - D: Then descriptor D will deactivate the VDAC
  //  - E: Then descriptor E will deactivate the LDMA

  // Create the array of linked descriptors
  static LDMA_Descriptor_t ldmaDescriptors[NUM_LDMA_DESCRIPTORS];

  // Descriptor A: ramp up from 0V to VDD/2 to avoid speaker popping sound
  ldmaDescriptors[LDMA_DESCRIPTOR_A_INDEX] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_LINKREL_M2P_BYTE(&BEGIN_RAMP[0],
                                     &VDAC0->CH0DATA,
                                     BEGIN_RAMP_SIZE,
                                     1);

  // The max transfer size is not enough to play the entire sine table so
  // we'll link the A-type descriptors together
  for (int i = 0; i < NUM_LDMA_DESCRIPTORS_B; i++)
  {
    if (i < NUM_LDMA_DESCRIPTORS_B-1)
    {
      // Each descriptor links to the next one
      ldmaDescriptors[LDMA_DESCRIPTOR_B_INDEX+i] = (LDMA_Descriptor_t)
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
      ldmaDescriptors[LDMA_DESCRIPTOR_B_INDEX+i] = (LDMA_Descriptor_t)
        LDMA_DESCRIPTOR_LINKREL_M2P_BYTE(&HONK_WAVEFORM[i*MAX_XFER_COUNT],
                                         &VDAC0->CH0DATA,
                                         remainingTransfers,
                                         -(NUM_LDMA_DESCRIPTORS_B-1));
      // The last A-type descriptor will decrease the loop count
      ldmaDescriptors[LDMA_DESCRIPTOR_B_INDEX+i].xfer.decLoopCnt = 1;
    }
    // Don't trigger interrupt when transfer is done
    ldmaDescriptors[LDMA_DESCRIPTOR_B_INDEX+i].xfer.doneIfs = 0;
    // Transfer halfwords (VDAC data register is 12 bits)
    ldmaDescriptors[LDMA_DESCRIPTOR_B_INDEX+i].xfer.size = ldmaCtrlSizeHalf;
  }

  // Descriptor C: ramp down from VDD/2 to 0V to avoid speaker popping sound
  ldmaDescriptors[LDMA_DESCRIPTOR_C_INDEX] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_LINKREL_M2P_BYTE(&END_RAMP[0],
                                     &VDAC0->CH0DATA,
                                     END_RAMP_SIZE,
                                     1);

  // Descriptor D: deactivate the VDAC
  ldmaDescriptors[LDMA_DESCRIPTOR_D_INDEX] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_SINGLE_WRITE(VDAC_CMD_CH0DIS,
                                 &VDAC0->CMD);
  ldmaDescriptors[LDMA_DESCRIPTOR_D_INDEX].wri.linkMode = ldmaLinkModeRel;
  ldmaDescriptors[LDMA_DESCRIPTOR_D_INDEX].wri.link = 1;
  ldmaDescriptors[LDMA_DESCRIPTOR_D_INDEX].wri.linkAddr = 1*4;

  // Descriptor E: deactivate the LDMA
  // TODO: ^
//  ldmaDescriptors[LDMA_DESCRIPTOR_E_INDEX] = (LDMA_Descriptor_t)
//      LDMA_DESCRIPTOR_SINGLE_WRITE(VDAC_CMD_CH0DIS,
//                                   &VDAC0->CMD);

  // Trigger when VDAC0_CH0DATA is empty
  LDMA_TransferCfg_t transferConfig = LDMA_TRANSFER_CFG_PERIPHERAL_LOOP(
    ldmaPeripheralSignal_VDAC0_CH0, NUM_LDMA_DESCRIPTORS_B_LOOPS);

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
