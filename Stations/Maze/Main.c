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

static volatile Position            position = {0};
static volatile MAZE_CONTROL_METHOD CTL_METHOD = DEFAULT_CTL_METHOD;

/**
 * @brief
 * @param
 */
void Maze_ChangeControlMethod(uint8_t *buffer) {
  uint8_t data = 0x00;

  if (buffer[0] != 1)
    return;

  memcpy(&data, buffer + DATA_OFFSET, 1);
  // Update Screen

  CTL_METHOD = data; // Set new method

  // Reset Game and Servo Positions
}

/**
 * @brief
 * @param buffer
 */
void Maze_MoveTo(uint8_t *buffer) {
  uint8_t legIdx = 0;

  // Verify length matches expected
  if (buffer[0] != POSITION_BYTE_SIZE)
    return;

  memcpy(&position, buffer + DATA_OFFSET, POSITION_BYTE_SIZE);

  StewartPlatform_Update(position.translation, position.quaternion);
  for (legIdx = 0; legIdx < Maestro_Channels; legIdx++) {
    Maestro_SetAngle(legIdx, legs[legIdx].servoAngle);
  }

  Maestro_WaitForIdle();
}

int main(void) {
  float angle = 0.0f;

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
        Maze_ChangeControlMethod(RX_Data_Buffer);
        break;
      case NEW_QUATERNION:
        Maze_MoveTo(RX_Data_Buffer);
        break;
      default:
        continue;
    }

    EnableInterrupts();

    HasNewData = false; // Clear data flag
  }
}
