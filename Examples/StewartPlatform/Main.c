#include <stdint.h>
#include <stdio.h>

#include "PLL.h"
#include "TivaWareFPU/fpu.h"

#include "CLI/CLI.h"

#include "StewartPlatform/StewartPlatform.h"

#define SYS_CLOCK 80e6

int main(void) {
  FPULazyStackingEnable(); // Enable Floating Point for use especially in Interrupts

  PLL_Init(); // Initialize the PLL

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  StewartPlatform_Init(0.0f, 0.0f, 0.0f, 0.0f); // Initialize stewart platform with variables from example

  while (1) {
    
  }
}
