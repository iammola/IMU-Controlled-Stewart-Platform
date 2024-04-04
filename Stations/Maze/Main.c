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
#include <stdio.h>
#include <string.h>

#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "Joystick/Joystick.h"
#include "Pololu Maestro/Maestro.h"
#include "RA8875/RA8875.h"
#include "StewartPlatform/StewartPlatform.h"
#include "Timer/Ping/Ping.h"
#include "Wireless/Wireless.h"

#include "Maze.h"

#define SYS_CLOCK 80e6

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

static volatile Position            position = {0};
static volatile bool                connectionState;
static volatile MAZE_CONTROL_METHOD CTL_METHOD;

void Maze_UpdateControlMethod(MAZE_CONTROL_METHOD newControl) {
  CTL_METHOD = newControl; // Set new method

  switch (CTL_METHOD) {
    case JOYSTICK_CTL_METHOD:
      Joystick_Enable();
      break;
    case IMU_CTL_METHOD:
      Joystick_Disable();
      break;
  }

  DisableInterrupts();
  Wireless_Transmit(CHANGE_CONTROL_METHOD_ACK, (uint8_t *)&CTL_METHOD, CHANGE_CONTROL_METHOD_LENGTH); // Send ACK
  EnableInterrupts();

  /* Clear section */
  RA8875_graphicsMode();
  RA8875_fillRect(CONTROL_METHOD_X, CONTROL_METHOD_Y, CONTROL_METHOD_WIDTH, CONTROL_METHOD_HEIGHT, RA8875_BLACK);

  /* Print text */
  RA8875_textMode();
  RA8875_textSetCursor(CONTROL_METHOD_X, CONTROL_METHOD_Y);
  RA8875_textEnlarge(0);
  RA8875_textTransparent(RA8875_WHITE);
  RA8875_textWrite(CTL_METHOD == JOYSTICK_CTL_METHOD ? "Control Method: Controller" : "Control Method: Glove", 0);
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

/**
 * @brief
 * @param connected
 */
void Maze_UpdateConnectedState(bool connected) {
  if (connectionState == connected)
    return;

  Ping_TimerReset();

  /* Clear section */
  RA8875_graphicsMode();
  RA8875_fillRect(CONNECTED_STATE_X, CONNECTED_STATE_Y, CONNECTED_STATE_WIDTH, CONNECTED_STATE_HEIGHT, RA8875_BLACK);

  /* Print text */
  RA8875_textMode();
  RA8875_textSetCursor(CONNECTED_STATE_X, CONNECTED_STATE_Y);
  RA8875_textEnlarge(0);
  RA8875_textTransparent(connected ? RA8875_GREEN : RA8875_RED);
  RA8875_textWrite(connected ? "Connected to Glove" : "Disconnected from Glove", 0);

  connectionState = connected;
}

void Ping_Handler(void) {
  Maze_UpdateConnectedState(false);
}

int main(void) {
  uint8_t dataLength;

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  DisableInterrupts(); // Disable interrupts until after config

  Ping_TimerInit((uint32_t)(SYS_CLOCK * 3));
  Wireless_Init(SYS_CLOCK, true);          // Initialize Wireless
  RA8875_begin(SYS_CLOCK, RA8875_800x480); // Init screen
  Joystick_Init(SYS_CLOCK, &position);     // Initialize Joystick
  Maestro_Init(SYS_CLOCK);                 // Initialize Maestro Controller

  StewartPlatform_Init(); // Initialize stewart platform

  RA8875_textMode();
  RA8875_textSetCursor(230, 10);
  RA8875_textEnlarge(1);
  RA8875_textTransparent(RA8875_WHITE);
  RA8875_textWrite("Initialized", 0);
  RA8875_textEnlarge(0);

  Maze_UpdateControlMethod(DEFAULT_CTL_METHOD);
  Maze_UpdateConnectedState(false);

  EnableInterrupts(); // Enable all interrupts

  while (1) {
    WaitForInterrupt();

    // Direct Joystick control
    if (CTL_METHOD == JOYSTICK_CTL_METHOD && position.isNew)
      Maze_MoveToPosition();

    // Wait for new data to be confirmed
    if (ReceivedCommands.ChangeControlMethod.inUse) {
      Maze_UpdateControlMethod(ReceivedCommands.ChangeControlMethod.data[0]);

      ReceivedCommands.ChangeControlMethod.inUse = false;
    }

    if (ReceivedCommands.NewPosition.inUse) {
      memcpy(&position, ReceivedCommands.NewPosition.data, sizeof(ReceivedCommands.NewPosition.data));
      position.isNew = true;
      Maze_MoveToPosition();

      ReceivedCommands.NewPosition.inUse = false;
      Maze_UpdateConnectedState(true);
    }
  }
}
