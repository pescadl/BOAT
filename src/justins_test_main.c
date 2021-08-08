
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"

#include "../inc/speaker.h"

/**************************************************************************//**
 * @brief
 *    Output a sine wave to DAC channel 0
 *****************************************************************************/
int main(void)
{
  // Chip errata
  CHIP_Init();

  // Init DCDC regulator with kit specific parameters
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
  EMU_DCDCInit(&dcdcInit);

  Speaker_Init();

  while (1) {
    EMU_EnterEM1(); // Enter EM1 (won't exit)
  }
}

