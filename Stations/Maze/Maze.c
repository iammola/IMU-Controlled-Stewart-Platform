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

#include "SysTick/SysTick.h"

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
 * @param
 */
static void Maze_DrawObstacles(void);

/**
 * @brief
 * @param SYS_CLOCK
 */
void Maze_Init(const uint32_t SYS_CLOCK) {
  RA8875_begin(SYS_CLOCK, RA8875_800x480); // Init screen
  Joystick_Init(SYS_CLOCK, &position);     // Initialize Joystick

  StewartPlatform_Init();  // Initialize stewart platform
  Maestro_Init(SYS_CLOCK); // Initialize Servo Maestro Controller

  Ping_TimerInit(SYS_CLOCK, false); // Init timer for 1 second

  Wireless_Init(SYS_CLOCK, true); // Initialize Wireless

  Maze_UpdateControlMethod(DEFAULT_CTL_METHOD); // Initialize and Send method
  Maze_UpdateConnectedState(DISCONNECTED);
  Maze_DrawObstacles();

  Maze_MoveToNeutralPosition();
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

  /* Clear section */
  RA8875_graphicsMode();
  RA8875_fillRect(CONTROL_METHOD_X, CONTROL_METHOD_Y, CONTROL_METHOD_WIDTH, CONTROL_METHOD_HEIGHT, RA8875_BLACK);

  /* Print text */
  RA8875_textMode();
  RA8875_textSetCursor(CONTROL_METHOD_X, CONTROL_METHOD_Y);
  RA8875_textEnlarge(0);
  RA8875_textTransparent(RA8875_WHITE);
  RA8875_textWrite(CTL_METHOD == JOYSTICK_CTL_METHOD ? "Control Method: Controller" : "Control Method: Glove", 0);

  Maze_MoveToNeutralPosition();

  SysTick_WaitCustom(10, -3);
  Wireless_Transmit(CHANGE_CONTROL_METHOD_ACK, (uint8_t *)&CTL_METHOD, CHANGE_CONTROL_METHOD_ACK_LENGTH); // Send ACK
}

/**
 * @brief
 * @param
 */
void Maze_MoveToNeutralPosition(void) {
  position.inUse = true;
  position.translation = (Coords){.x = 0.0f, .y = 0.0f, .z = 0.0f};
  position.quaternion = (Quaternion){.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};

  Maze_MoveToPosition();
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
static void Maze_DrawObstacles(void) {
  uint8_t  lineIdx;
  uint16_t color;
  int16_t  startX, startY;

  static const struct {
    uint32_t col;
    uint32_t row;
    bool     isHorizontal;
    bool     isGate;
  } lines[] = {
  // Start Gate
      {.col = 5, .row = 1,  .isHorizontal = true,  .isGate = true },
 // 1st row
      {.col = 2, .row = 1,  .isHorizontal = false, .isGate = false},
      {.col = 5, .row = 1,  .isHorizontal = false, .isGate = false},
      {.col = 8, .row = 1,  .isHorizontal = false, .isGate = false},
 // 2nd row
      {.col = 3, .row = 2,  .isHorizontal = true,  .isGate = false},
      {.col = 5, .row = 2,  .isHorizontal = true,  .isGate = false},
      {.col = 6, .row = 2,  .isHorizontal = true,  .isGate = false},
      {.col = 8, .row = 2,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 2,  .isHorizontal = false, .isGate = false},
      {.col = 4, .row = 2,  .isHorizontal = false, .isGate = false},
      {.col = 7, .row = 2,  .isHorizontal = false, .isGate = false},
 // 3rd row
      {.col = 1, .row = 3,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 3,  .isHorizontal = false, .isGate = false},
      {.col = 5, .row = 3,  .isHorizontal = true,  .isGate = false},
      {.col = 6, .row = 3,  .isHorizontal = true,  .isGate = false},
      {.col = 7, .row = 3,  .isHorizontal = true,  .isGate = false},
      {.col = 8, .row = 3,  .isHorizontal = true,  .isGate = false},
 // 4th row
      {.col = 2, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 5, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 6, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 7, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 8, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 9, .row = 4,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 4,  .isHorizontal = false, .isGate = false},
 // 5th row
      {.col = 1, .row = 5,  .isHorizontal = true,  .isGate = false},
      {.col = 2, .row = 5,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 5,  .isHorizontal = true,  .isGate = false},
      {.col = 5, .row = 5,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 5,  .isHorizontal = false, .isGate = false},
      {.col = 7, .row = 5,  .isHorizontal = false, .isGate = false},
      {.col = 8, .row = 5,  .isHorizontal = false, .isGate = false},
      {.col = 9, .row = 5,  .isHorizontal = false, .isGate = false},
 // 6th row
      {.col = 2, .row = 6,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 6,  .isHorizontal = true,  .isGate = false},
      {.col = 5, .row = 6,  .isHorizontal = true,  .isGate = false},
      {.col = 6, .row = 6,  .isHorizontal = true,  .isGate = false},
      {.col = 8, .row = 6,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 6,  .isHorizontal = false, .isGate = false},
      {.col = 5, .row = 6,  .isHorizontal = false, .isGate = false},
      {.col = 9, .row = 6,  .isHorizontal = false, .isGate = false},
 // 7th row
      {.col = 1, .row = 7,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 7,  .isHorizontal = true,  .isGate = false},
      {.col = 6, .row = 7,  .isHorizontal = true,  .isGate = false},
      {.col = 7, .row = 7,  .isHorizontal = true,  .isGate = false},
      {.col = 2, .row = 7,  .isHorizontal = false, .isGate = false},
      {.col = 4, .row = 7,  .isHorizontal = false, .isGate = false},
      {.col = 6, .row = 7,  .isHorizontal = false, .isGate = false},
      {.col = 8, .row = 7,  .isHorizontal = false, .isGate = false},
      {.col = 9, .row = 7,  .isHorizontal = false, .isGate = false},
 // 8th row
      {.col = 3, .row = 8,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 8,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 8,  .isHorizontal = false, .isGate = false},
      {.col = 6, .row = 8,  .isHorizontal = false, .isGate = false},
      {.col = 7, .row = 8,  .isHorizontal = false, .isGate = false},
      {.col = 9, .row = 8,  .isHorizontal = false, .isGate = false},
 // 9th row
      {.col = 1, .row = 9,  .isHorizontal = true,  .isGate = false},
      {.col = 2, .row = 9,  .isHorizontal = true,  .isGate = false},
      {.col = 3, .row = 9,  .isHorizontal = true,  .isGate = false},
      {.col = 4, .row = 9,  .isHorizontal = true,  .isGate = false},
      {.col = 7, .row = 9,  .isHorizontal = true,  .isGate = false},
      {.col = 8, .row = 9,  .isHorizontal = true,  .isGate = false},
 //  End Gate
      {.col = 5, .row = 10, .isHorizontal = true,  .isGate = true },
  };
  const uint8_t linesCount = sizeof(lines) / sizeof(lines[0]);

  /* Clear section */
  RA8875_graphicsMode();

  RA8875_fillRect(MAZE_X, MAZE_Y, MAZE_WIDTH, MAZE_HEIGHT, RA8875_BLACK); // Clear Screen

  RA8875_drawRect(MAZE_X, MAZE_Y, MAZE_WIDTH, MAZE_HEIGHT, RA8875_WHITE); // Maze Box
  for (lineIdx = 0; lineIdx < linesCount; lineIdx++) {                    // Draw lines
    if (lines[lineIdx].row > MAZE_ROWS_COUNT) {
      startY = MAZE_HEIGHT - 1 + MAZE_Y; // Draw on box outer line
    } else {
      startY = (int16_t)(((lines[lineIdx].row - 1) * MAZE_CELL_SIZE) + MAZE_Y);
    }

    startX = (int16_t)(((lines[lineIdx].col - 1) * MAZE_CELL_SIZE) + MAZE_X);
    color = lines[lineIdx].isGate ? RA8875_BLACK : RA8875_WHITE; // Draw walls in white, gates in black for "transparent"

    if (lines[lineIdx].isHorizontal) {
      RA8875_drawFastHLine(startX, startY, MAZE_CELL_SIZE, color);
    } else {
      RA8875_drawFastVLine(startX, startY, MAZE_CELL_SIZE, color);
    }
  }
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
