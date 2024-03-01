#include <stdint.h>

#include "PLL.h"
#include "SysTick.h"

#include "CLI/CLI.h"

#include "IMU/IMU.h"
#include "StewartPlatform/StewartPlatform.h"
#include "TivaWareFPU/fpu.h" // FPU Driver from TI

#define SYS_CLOCK 80e6

int main(void) {
  FPULazyStackingEnable(); // Enable Floating Point for use especially in Interrupts

  SysTick_Init(); // Initialize SysTick
  PLL_Init();     // Initialize the PLL

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  IMU_Init(SYS_CLOCK, 1e6); // Initialize the IMU with the SYS_CLOCK and to communicate at 1MHz
  // StewartPlatform_Init(6.2f, 5, 5.08f, 10, 0.2269f, 0.2269f, 0); // Initialize stewart platform with variables from example

  while (1)
    ;
}
