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

#include <string.h>

#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "IMU/IMU.h"
#include "Wireless/Wireless.h"

#include "Glove.h"

#define SYS_CLOCK              80e6
#define JOYSTICK_SAMPLING_RATE 100

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

volatile bool                HasNewQuat = false;
volatile MAZE_CONTROL_METHOD CTL_METHOD = DEFAULT_CTL_METHOD;

int main(void) {
  Position position = {0};

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  Wireless_Init(SYS_CLOCK);
  IMU_Init(SYS_CLOCK, &position);

  switch (CTL_METHOD) {
    case JOYSTICK_CTL_METHOD:
      IMU_Disable();
      break;
    case IMU_CTL_METHOD:
      IMU_Enable();
      break;
  }

  while (1) {
    WaitForInterrupt();

    if (!position.isNew)
      continue;

    TX_Data_Buffer[0] = NEW_POSITION;
    memcpy(TX_Data_Buffer + 1, &position, POSITION_BYTE_SIZE);

    HC12_SendData(TX_Data_Buffer, POSITION_BYTE_SIZE + 1);

    position.isNew = false;
  }
}
