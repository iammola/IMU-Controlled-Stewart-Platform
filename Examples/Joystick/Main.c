/**
 * @file Main.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "CLI/CLI.h"
#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "Joystick/Joystick.h"

#define SYS_CLOCK 80e6

void WaitForInterrupt(void);
void DisableInterrupts(void);
void EnableInterrupts(void);

static char text[CLI_TXT_BUF] = "";

int main(void) {
  Position position = {0};

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  Joystick_Init(SYS_CLOCK, &position);
  Joystick_Enable();

  while (1) {
    WaitForInterrupt();

    position.inUse = true;
    snprintf(text, CLI_TXT_BUF, "%0.6f %0.6f %0.6f %0.6f\n", position.quaternion.w, position.quaternion.x, position.quaternion.y,
             position.quaternion.z);
    position.inUse = false;

    CLI_Write(text);
  }
}
