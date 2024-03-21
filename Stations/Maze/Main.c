/**
 * @file Main.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <math.h>
#include <string.h>

#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "HC-12/HC-12.h"
#include "Pololu Maestro/Maestro.h"
#include "StewartPlatform/StewartPlatform.h"

#define SYS_CLOCK 80e6

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

static const StewartCoords translation = {0.0f};

static float      angle = 0.0f;
static Quaternion stewartQuaternion = {0.0f};

int main(void) {
  uint8_t legIdx = 0;

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  HC12_Init();
  HC12_Config(SYS_CLOCK, BAUD_115200, TX_20dBm); // Use 115200 bps, 20 dBm

  Maestro_Init(SYS_CLOCK);                      // Initialize Maestro Controller
  StewartPlatform_Init(0.0f, 0.0f, 0.0f, 0.0f); // Initialize stewart platform

  while (1) {
    WaitForInterrupt();

    // Wait for new data to be confirmed
    if (!HasNewData)
      continue;

    DisableInterrupts();

    memcpy(&angle, RX_Data_Buffer, 4); // Read only 4 bytes for angle float

    stewartQuaternion = normalizeQuaternion(-13.0f, -cosf(angle), sinf(angle), 0.0f);
    StewartPlatform_Update(translation, stewartQuaternion);

    for (legIdx = 0; legIdx < Maestro_Channels; legIdx++) {
      Maestro_SetAngle(legIdx, legs[legIdx].servoAngle);
      Maestro_WaitForIdle();
    }

    HasNewData = false; // Clear data flag
    EnableInterrupts();
  }
}
