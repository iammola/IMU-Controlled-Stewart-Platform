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

#include "CLI/CLI.h"
#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "Joystick/Joystick.h"
#include "Quaternion/Quaternion.h"

#define SYS_CLOCK   80e6
#define SAMPLE_RATE 250

void WaitForInterrupt(void);
void DisableInterrupts(void);
void EnableInterrupts(void);

static char text[CLI_TXT_BUF] = "";

int main(void) {
  Quaternion quat = {0};

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  Joystick_Init(SYS_CLOCK, SAMPLE_RATE);

  while (1) {
    WaitForInterrupt();

    quat = normalizeQuaternion(-13.0f, /* 1.0f - */ -cosf(coords.angle), /* 1.0f - */ sinf(coords.angle), 0.0f);

    // snprintf(text, CLI_TXT_BUF, "%0.6f %0.6f %0.6f %0.6f", -cosf(coords.angle), VRx, sinf(coords.angle), VRy);
    snprintf(text, CLI_TXT_BUF, "%0.6f %0.6f %0.6f %0.6f", quat.w, quat.x, quat.y, quat.z);

    DisableInterrupts();
    CLI_Write(text);
    CLI_Write("\n");
    EnableInterrupts();
  }
}
