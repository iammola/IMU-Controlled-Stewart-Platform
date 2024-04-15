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
#include <stdio.h>

#include "Maze.h"

#include "SysTick/SysTick.h"

#include "Joystick/Joystick.h"
#include "Pololu Maestro/Maestro.h"
#include "RA8875/RA8875.h"
#include "StewartPlatform/StewartPlatform.h"
#include "Timer/Ping/Ping.h"
#include "Timer/RTC/RTC.h"
#include "Wireless/Wireless.h"

#define RTC_START_BTN              (1 << 4) // PF4 - SW1
#define RTC_STOP_BTN               (1 << 0) // PF0 - SW2
#define RTC_BTN_PCTL_M             (unsigned)(GPIO_PCTL_PF0_M | GPIO_PCTL_PF4_M)
#define RTC_BTN_INTERRUPT_PRIORITY 6

volatile TIME                time = {0};
volatile MAZE_CONTROL_METHOD CTL_METHOD;
volatile Position            position = {0};
volatile CONNECTED_STATE     connectionState;

static void Maze_DrawObstacles(void);
static void Maze_RTCButtonInit(void);

void GPIOF_Handler(void);

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

  RTC_TimerInit(SYS_CLOCK); // Initialize RTC Timer
  Maze_RTCButtonInit();     // Initialize RTC Trigger Buttons

  Wireless_Init(SYS_CLOCK, true); // Initialize Wireless

  time = (TIME){.milliseconds = 0, .seconds = 0, .minutes = 0, .updated = false}; // Init RTC timer

  Maze_UpdateControlMethod(DEFAULT_CTL_METHOD); // Initialize with default control method (will send to glove if to enable IMU)
  Maze_UpdateConnectedState(DISCONNECTED);      // Print Disconnected Glove state
  Maze_DrawObstacles();                         // Draw Maze board and Obstacles
  Maze_UpdateGameTime();                        // Print Game Time

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

  SysTick_WaitCustom(50, -3);                                                                             // 100ms delay
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
void Maze_UpdateGameTime(void) {
#define TEXT_BUF_LEN 10
  static char timeText[TEXT_BUF_LEN] = "";

  /* Clear section */
  RA8875_graphicsMode();
  RA8875_fillRect(SCREEN_TIME_X + 87 /* Start of Minute number */, SCREEN_TIME_Y, SCREEN_TIME_WIDTH, SCREEN_TIME_HEIGHT, RA8875_BLACK);

  /* Print text */
  RA8875_textMode();
  RA8875_textTransparent(RA8875_WHITE);

  RA8875_textSetCursor(SCREEN_TIME_X, SCREEN_TIME_Y + 10 /* Approximate vertical center of enlarged time number */);
  RA8875_textEnlarge(0);
  RA8875_textWrite("Game Time: ", 0);

  snprintf(timeText, TEXT_BUF_LEN, "%02d:%02d", time.minutes, time.seconds);
  RA8875_textSetYCursor(SCREEN_TIME_Y); // Change only Y position of cursor
  RA8875_textEnlarge(1);                // Print in larger font size
  RA8875_textWrite(timeText, 0);

  snprintf(timeText, TEXT_BUF_LEN, ".%03d ms", time.milliseconds);
  RA8875_textSetYCursor(SCREEN_TIME_Y + 10); // Change only Y position back to level of title text
  RA8875_textEnlarge(0);                     // Print in normal font size
  RA8875_textWrite(timeText, 0);
#undef TEXT_BUF_LEN
}

/**
 * @brief
 * @param
 */
static void Maze_RTCButtonInit(void) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // Enable Port F

  while ((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R5) == 0x00) { // Wait for Module ready
  }

  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;              // unlock GPIO Port F
  GPIO_PORTF_CR_R = RTC_START_BTN | RTC_STOP_BTN; // allow changes to buttons

  GPIO_PORTF_DIR_R &= (unsigned)~(RTC_START_BTN | RTC_STOP_BTN);   // configure buttons as inputs
  GPIO_PORTF_PUR_R |= RTC_START_BTN | RTC_STOP_BTN;                // pull-up buttons
  GPIO_PORTF_DEN_R |= RTC_START_BTN | RTC_STOP_BTN;                // enable digital I/O on buttons
  GPIO_PORTF_AMSEL_R &= (unsigned)~(RTC_START_BTN | RTC_STOP_BTN); // disable analog on buttons
  GPIO_PORTF_AFSEL_R &= (unsigned)~(RTC_START_BTN | RTC_STOP_BTN); // disable alternate functions onbuttons
  GPIO_PORTF_PCTL_R &= ~RTC_BTN_PCTL_M;                            // no peripheral functions with disabled alt. functions

  GPIO_PORTF_IS_R &= (unsigned)~(RTC_START_BTN | RTC_STOP_BTN);  // Configure for Edge-Detect on (unsigned)
  GPIO_PORTF_IBE_R &= (unsigned)~(RTC_START_BTN | RTC_STOP_BTN); // Allow GPIOIEV register to control interrupt
  GPIO_PORTF_IEV_R &= (unsigned)~(RTC_START_BTN | RTC_STOP_BTN); // Trigger falling edge on PF0
  GPIO_PORTF_IM_R = RTC_START_BTN | RTC_STOP_BTN;                // Allow interrupts to be sent

  NVIC_EN0_R |= NVIC_EN0_INT30; // Enable Interrupt 30 for GPIO Port F
  NVIC_PRI7_R = (NVIC_PRI7_R & (unsigned)~NVIC_PRI7_INT30_M) | (RTC_BTN_INTERRUPT_PRIORITY << NVIC_PRI7_INT30_S); // Set Priority
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
inline void Ping_Handler(void) {
  Maze_UpdateConnectedState(DISCONNECTED);
}

/**
 * @brief
 * @param
 */
inline void RTC_Handler(void) {
  ++time.milliseconds;

  if (time.milliseconds == 1000) { // Milliseconds max 1000 ms
    ++time.seconds;
    time.milliseconds = 0;
  }

  if (time.seconds == 60) { // Seconds max 60 sec
    ++time.minutes;
    time.seconds = 0;
  }

  if (time.minutes == 60) { // Minutes max = 60 min.
    time.minutes = 0;
  }

  if ((time.milliseconds % 45) == 0) { // Update screen every 75ms
    time.updated = true;
  }
}

void GPIOF_Handler(void) {
  switch (GPIO_PORTF_MIS_R & (RTC_START_BTN | RTC_STOP_BTN)) {
    case RTC_START_BTN:
      GPIO_PORTF_ICR_R |= RTC_START_BTN; // Clear START interrupt
      if (!time.running) {
        RTC_TimerStart();
        time = (TIME){.milliseconds = 0, .seconds = 0, .minutes = 0, .updated = true, .running = true}; // Reset counter
      }
      break;
    case RTC_STOP_BTN:
      GPIO_PORTF_ICR_R |= RTC_STOP_BTN; // Clear STOP interrupt
      if (time.running) {
        RTC_TimerStop();
        time.running = false;
      }
      break;
    default:
      return;
  }
}
