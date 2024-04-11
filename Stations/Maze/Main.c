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

#include "Wireless/Wireless.h"

#include "Maze.h"

#define SYS_CLOCK 80e6

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

int main(void) {
  DisableInterrupts(); // Disable interrupts until after config

  PLL_Init();              // Init PLL for 80MHz
  FPULazyStackingEnable(); // Enable Floating Point co-pro
  Maze_Init(SYS_CLOCK);

  EnableInterrupts(); // Enable all interrupts

  while (1) {
    WaitForInterrupt();

    // Direct Joystick control
    if (CTL_METHOD == JOYSTICK_CTL_METHOD && position.count > 0) {
      position.inUse = true;
      Maze_MoveToPosition();
      position.count = 0;
    }

    if (ReceivedCommands.ChangeControlMethod.isNew) {
      ReceivedCommands.ChangeControlMethod.inUse = false;

      Maze_UpdateControlMethod(ReceivedCommands.ChangeControlMethod.data[0]);

      ReceivedCommands.ChangeControlMethod.isNew = ReceivedCommands.ChangeControlMethod.inUse = false;
    }

    if (ReceivedCommands.NewPosition.isNew) {
      ReceivedCommands.NewPosition.inUse = position.inUse = true;
      memcpy(&position, ReceivedCommands.NewPosition.data, sizeof(ReceivedCommands.NewPosition.data));

      Maze_MoveToPosition();
      Maze_UpdateConnectedState(CONNECTED);

      ReceivedCommands.NewPosition.isNew = ReceivedCommands.NewPosition.inUse = false;
    }
  }
}
