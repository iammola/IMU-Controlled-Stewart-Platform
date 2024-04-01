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

static volatile MAZE_CONTROL_METHOD CTL_METHOD = DEFAULT_CTL_METHOD;

static Position   position = {0};
static Quaternion positionFromOffset = {0};
static uint8_t    positionInBytes[POSITION_BYTE_SIZE] = {0};

int main(void) {
  const Quaternion positionOffset = QuaternionInverse(0.628237f, 0.014747f, -0.585671f, -0.512185f);

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  Wireless_Init(SYS_CLOCK, false);
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

    // Get the diff from the offset quaternion
    positionFromOffset = QuaternionMultiply(position.quaternion, positionOffset);

    memcpy(positionInBytes, &positionFromOffset, POSITION_BYTE_SIZE);
    Wireless_Transmit(NEW_POSITION, positionInBytes, POSITION_BYTE_SIZE);

    position.isNew = false;
  }
}
