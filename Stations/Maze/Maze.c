/**
 * @file Maze.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Maze.h"

#include "Joystick/Joystick.h"
#include "Pololu Maestro/Maestro.h"
#include "RA8875/RA8875.h"
#include "StewartPlatform/StewartPlatform.h"
#include "Timer/Ping/Ping.h"
#include "Wireless/Wireless.h"

volatile MAZE_CONTROL_METHOD CTL_METHOD;
volatile Position            position = {0};
volatile CONNECTED_STATE     connectionState;

/**
 * @brief
 * @param SYS_CLOCK
 */
void Maze_Init(const uint32_t SYS_CLOCK) {
  Wireless_Init(SYS_CLOCK, true); // Initialize Wireless

  RA8875_begin(SYS_CLOCK, RA8875_800x480); // Init screen
  Joystick_Init(SYS_CLOCK, &position);     // Initialize Joystick

  StewartPlatform_Init();  // Initialize stewart platform
  Maestro_Init(SYS_CLOCK); // Initialize Servo Maestro Controller

  Ping_TimerInit((uint32_t)(SYS_CLOCK * 3), false); // Init timer for 3 seconds

  Maze_UpdateControlMethod(DEFAULT_CTL_METHOD); // Initialize and Send method
  Maze_UpdateConnectedState(DISCONNECTED);
}

/**
 * @brief
 * @param newControl
 */
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

  Wireless_Transmit(CHANGE_CONTROL_METHOD_ACK, (uint8_t *)&CTL_METHOD, CHANGE_CONTROL_METHOD_LENGTH); // Send ACK

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

  if (!position.inUse)
    return;

  StewartPlatform_Update(position.translation, position.quaternion);
  for (legIdx = 0; legIdx < LEGS_COUNT; legIdx++) {
    // The servos on the odd channels are reversed, so -ve angles to rotate up
    Maestro_SetAngle(legIdx, legs[legIdx].servoAngle * ((legIdx % 2 == 0) ? 1.0f : -1.0f));
  }

  position.inUse = false;

  Maestro_WaitForIdle();
}

/**
 * @brief
 * @param connected
 */
void Maze_UpdateConnectedState(CONNECTED_STATE connected) {
  if (connected == DISCONNECTED) {
    // Only disable timer if disconnected so a CONNECTED event
    // would re-enable
    Ping_TimerDisable();
  } else {
    Ping_TimerEnable(); // Enable Timer to check connection state periodically
    Ping_TimerReset();
  }

  if (connectionState == connected) {
    // Return early to avoid re-printing state to screen
    return;
  }

  /* Clear section */
  RA8875_graphicsMode();
  RA8875_fillRect(CONNECTED_STATE_X, CONNECTED_STATE_Y, CONNECTED_STATE_WIDTH, CONNECTED_STATE_HEIGHT, RA8875_BLACK);

  /* Print text */
  RA8875_textMode();
  RA8875_textSetCursor(CONNECTED_STATE_X, CONNECTED_STATE_Y);
  RA8875_textEnlarge(0);

  if (connected == CONNECTED) {
    RA8875_textTransparent(RA8875_GREEN);
    RA8875_textWrite("Connected to Glove", 0);
  } else {
    RA8875_textTransparent(RA8875_RED);
    RA8875_textWrite("Disconnected from Glove", 0);
  }

  connectionState = connected;
}

/**
 * @brief
 * @param
 */
inline void Maze_ConfirmedInitialized(void) {
  RA8875_textMode();
  RA8875_textSetCursor(230, 10);
  RA8875_textEnlarge(1);
  RA8875_textTransparent(RA8875_WHITE);
  RA8875_textWrite("Initialized", 0);
  RA8875_textEnlarge(0);
}

/**
 * @brief
 * @param
 */
inline void Ping_Handler(void) {
  Maze_UpdateConnectedState(DISCONNECTED);
}
