#include <stdint.h>
#include <stdio.h>

#include "PLL.h"
#include "SysTick.h"

#include "CLI/CLI.h"
#include "IMU.h"

#define SYS_CLOCK   80e6
#define CLI_TXT_BUF 500

static char text[CLI_TXT_BUF] = "";

int main(void) {
  SysTick_Init(); // Initialize SysTick
  PLL_Init();     // Initialize the PLL

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_7_8, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  IMU_Init(SYS_CLOCK, 1e6); // Initialize the IMU with the SYS_CLOCK and to communicate at 1MHz

  while (1) {
    SysTick_Wait10ms(500);
  }
}
