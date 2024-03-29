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

#include "Joystick/Joystick.h"
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
 * @param dataLength
 * @param buffer
 */
void Maze_ChangeControlMethod(uint8_t dataLength, uint8_t *buffer) {
  uint8_t data = 0x00;

  if (dataLength != 0x01)
    return;

  memcpy(&data, buffer, 1);
  // Update Screen

  if (data == CTL_METHOD)
    return;

  CTL_METHOD = data; // Set new method

  if (CTL_METHOD == JOYSTICK_CTL_METHOD)
    Joystick_Enable();
  else
    Joystick_Disable();

  // Reset Game and Servo Positions
}

/**
 * @brief
 * @param dataLength
 * @param buffer
 */
void Maze_ReadNewPosition(uint8_t dataLength, uint8_t *buffer) {
  // Verify length matches expected
  if (dataLength != POSITION_BYTE_SIZE)
    return;

  memcpy(&position, buffer, POSITION_BYTE_SIZE);
  position.isNew = true;

  Maze_MoveToPosition();
}

/**
 * @brief
 * @param
 */
void Maze_MoveToPosition(void) {
  uint8_t legIdx = 0;

  if (!position.isNew)
    return;

  StewartPlatform_Update(position.translation, position.quaternion);
  for (legIdx = 0; legIdx < LEGS_COUNT; legIdx++) {
    // The servos on the odd channels are reversed, so -ve angles to rotate up
    Maestro_SetAngle(legIdx, legs[legIdx].servoAngle * ((legIdx % 2 == 0) ? 1.0f : -1.0f));
  }

  position.isNew = false;

  Maestro_WaitForIdle();
}

int main(void) {
  COMMAND cmd = 0x00;
  uint8_t dataLength = 0x00;

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  DisableInterrupts(); // Disable interrupts until after config

  Wireless_Init(SYS_CLOCK, true);      // Initialize Wireless
  Maestro_Init(SYS_CLOCK);             // Initialize Maestro Controller
  StewartPlatform_Init();              // Initialize stewart platform
  Joystick_Init(SYS_CLOCK, &position); // Initialize Joystick

  if (CTL_METHOD == JOYSTICK_CTL_METHOD) // Enable Joystick Handler if default
    Joystick_Enable();

  EnableInterrupts(); // Enable all interrupts

  while (1) {
    WaitForInterrupt();

    // Direct Joystick control
    if (CTL_METHOD == JOYSTICK_CTL_METHOD && position.isNew)
      Maze_MoveToPosition();

    // Wait for new data to be confirmed
    if (!HasNewData)
      continue;

    DisableInterrupts();

    cmd = RX_Data_Buffer[0];
    dataLength = RX_Data_Buffer[1];

    switch (cmd) {
      case CHANGE_CONTROL_METHOD:
        Maze_ChangeControlMethod(dataLength, RX_Data_Buffer + DATA_OFFSET);
        break;
      case NEW_POSITION:
        Maze_ReadNewPosition(dataLength, RX_Data_Buffer + DATA_OFFSET);
        break;
    }

    EnableInterrupts();

    HasNewData = false; // Clear data flag
  }
}
