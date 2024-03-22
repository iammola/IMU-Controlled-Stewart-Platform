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
#include "Joystick/Joystick.h"
#include "Wireless/Wireless.h"

#include "Glove.h"

#define SYS_CLOCK              80e6
#define JOYSTICK_SAMPLING_RATE 100

#define FLOAT_SIZE 4

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

volatile bool                HasNewQuat = false;
volatile MAZE_CONTROL_METHOD CTL_METHOD = DEFAULT_CTL_METHOD;

int main(void) {
  Quaternion quaternion = {0};

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  Wireless_Init(SYS_CLOCK);

  Joystick_Init(SYS_CLOCK, JOYSTICK_SAMPLING_RATE, &quaternion, &HasNewQuat);
  IMU_Init(SYS_CLOCK, &quaternion, &HasNewQuat);

  if (CTL_METHOD == JOYSTICK_CTL_METHOD) {
    Joystick_Enable();
  } else if (CTL_METHOD == IMU_CTL_METHOD) {
    IMU_Enable();
  }

  switch (CTL_METHOD) {
    case JOYSTICK_CTL_METHOD:
      Joystick_Enable();
      break;
    case IMU_CTL_METHOD:
      IMU_Enable();
      break;
  }

  while (1) {
    WaitForInterrupt();

    if (!HasNewQuat)
      continue;

    TX_Data_Buffer[0] = NEW_QUATERNION;
    memcpy(TX_Data_Buffer + 1, &quaternion, 16); // Copies in the order of W-X-Y-Z into buffer

    HC12_SendData(TX_Data_Buffer, 17);

    HasNewQuat = false;
  }
}
