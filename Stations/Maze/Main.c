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
#include <stdbool.h>
#include <string.h>

#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "Pololu Maestro/Maestro.h"
#include "StewartPlatform/StewartPlatform.h"
#include "Wireless/Wireless.h"

#include "Maze.h"

#define SYS_CLOCK 80e6

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

MAZE_CONTROL_METHOD CTL_METHOD = DEFAULT_CTL_METHOD;

/**
 * @brief
 * @param
 */
void Maze_ChangeControlMethod(uint8_t *RX_Data_Buffer) {
  // Update Screen
  CTL_METHOD = RX_Data_Buffer[2];
  // Reset Game and Servo Positions
}

int main(void) {
  uint8_t       legIdx = 0;
  float         angle = 0.0f;
  StewartCoords translation = {0};

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  Wireless_Init(SYS_CLOCK);

  Maestro_Init(SYS_CLOCK); // Initialize Maestro Controller
  StewartPlatform_Init();  // Initialize stewart platform

  while (1) {
    WaitForInterrupt();

    // Wait for new data to be confirmed
    if (!HasNewData)
      continue;

    DisableInterrupts();

    switch (RX_Data_Buffer[1]) {
      case CHANGE_CONTROL_METHOD:
        /* code */
        break;
      case NEW_QUATERNION:
        break;
      default:
        continue;
    }

    EnableInterrupts();

    memcpy(&angle, RX_Data_Buffer, 4); // Read only 4 bytes for angle float

    stewartQuaternion = normalizeQuaternion(-13.0f, -cosf(angle), sinf(angle), 0.0f);
    StewartPlatform_Update(translation, stewartQuaternion);

    for (legIdx = 0; legIdx < Maestro_Channels; legIdx++) {
      Maestro_SetAngle(legIdx, legs[legIdx].servoAngle);
      Maestro_WaitForIdle();
    }

    HasNewData = false; // Clear data flag
  }
}
