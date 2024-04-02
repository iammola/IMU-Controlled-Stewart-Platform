#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "PLL/PLL.h"
#include "tm4c123gh6pm.h"

#include "CLI/CLI.h"
#include "Wireless/Wireless.h"

#define SYS_CLOCK 80e6

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

static char text[CLI_TXT_BUF] = "";

int main(void) {
  uint32_t idx = 0;
  PLL_Init(); // Initialize PLL

  Wireless_Init(SYS_CLOCK, true);
  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT);

  while (1) {
    WaitForInterrupt();
    DisableInterrupts();

    // [0]=CMD, [1]=LENGTH
    for (idx = 0; idx < RX_Data_Buffer[1] + 2; idx++) {
      snprintf(text, CLI_TXT_BUF, "0x%02X", RX_Data_Buffer[idx]);

      if (idx > 0)
        CLI_Write(" ");
      CLI_Write(text);
    }

    if (idx > 0)
      CLI_Write("\n");

    EnableInterrupts();
  }
}
