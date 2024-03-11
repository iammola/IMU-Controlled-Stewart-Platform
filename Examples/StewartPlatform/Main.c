#include <stdint.h>
#include <stdio.h>

#include "FPU/fpu.h"
#include "PLL.h"

#include "CLI/CLI.h"

#include "IMU/IMU.h"
#include "StewartPlatform/StewartPlatform.h"

#define SYS_CLOCK 80e6

static char         text[CLI_TXT_BUF] = "";
static const Coords translation = {0.0f};

int main(void) {
  Quaternion stewartQuaternion = {0.0f};
  FPULazyStackingEnable(); // Enable Floating Point for use especially in Interrupts

  PLL_Init(); // Initialize the PLL

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  IMU_Init(SYS_CLOCK);                          // Initialize IMU
  StewartPlatform_Init(0.0f, 0.0f, 0.0f, 0.0f); // Initialize stewart platform

  while (1) {
    if (!HasNewIMUAngles)
      continue;

    stewartQuaternion.w = quaternion.element.w;
    stewartQuaternion.x = quaternion.element.x;
    stewartQuaternion.y = quaternion.element.y;
    stewartQuaternion.z = quaternion.element.z;

    StewartPlatform_Update(translation, stewartQuaternion);

    HasNewIMUAngles = false;
  }
}
