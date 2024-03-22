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

#include "HC-12/HC-12.h"
#include "IMU/IMU.h"
#include "Joystick/Joystick.h"

#define SYS_CLOCK              80e6
#define JOYSTICK_SAMPLING_RATE 100

#define FLOATS_COUNT 1 // Just for the angle
#define DATA_SIZE    (FLOATS_COUNT * 4)

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

static uint8_t data[DATA_SIZE] = {0};

int main(void) {
  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  HC12_Init();
  HC12_Config(SYS_CLOCK, BAUD_115200, TX_20dBm); // Use 115200 bps, 20 dBm

  Joystick_Init(SYS_CLOCK, JOYSTICK_SAMPLING_RATE);
  IMU_Init(SYS_CLOCK);

  /* TODO: Add button trigger to enable one of the two input methods. */

  while (1) {
    WaitForInterrupt();

    if (!HasNewJoystickCoords)
      continue;

    memcpy(data, &coords.angle, DATA_SIZE);
    HC12_SendData(data, DATA_SIZE);

    HasNewJoystickCoords = false;
  }
}
