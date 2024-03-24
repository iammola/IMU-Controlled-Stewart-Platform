/**
 * @file Main.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------

//      <o> CTL: Control Method
//              < 0=>  0: ICM-20948 MU
//              < 1=>  1: Joystick
//          <i> Input control method
//
#define CONTROL_METHOD 1

//-------- <<< end of configuration section >>> ------------------------------
#include <stdint.h>
#include <stdio.h>

#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "IMU/IMU.h"
#include "Joystick/Joystick.h"
#include "Pololu Maestro/Maestro.h"
#include "StewartPlatform/StewartPlatform.h"

#define SYS_CLOCK 80e6

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

int main(void) {
  uint8_t  legIdx = 0;
  Position position = {0};

  PLL_Init();              // Initialize the PLL
  FPULazyStackingEnable(); // Enable Floating Point

#if CONTROL_METHOD == 0
  IMU_Init(SYS_CLOCK, &position); // Initialize IMU
  IMU_Enable();
#elif CONTROL_METHOD == 1
  Joystick_Init(SYS_CLOCK, 100, &position); // Initialize Joystick
  Joystick_Enable();
#endif
  Maestro_Init(SYS_CLOCK); // Initialize Maestro Controller
  StewartPlatform_Init();  // Initialize stewart platform

  for (legIdx = 0; legIdx < Maestro_Channels; legIdx++) { // Set Initial angles
    Maestro_SetAngle(legIdx, legs[legIdx].servoAngle);
  }

  while (1) {
    WaitForInterrupt();

    if (!position.isNew)
      continue;

    DisableInterrupts();
    StewartPlatform_Update(position.translation, position.quaternion);

    for (legIdx = 0; legIdx < Maestro_Channels; legIdx++) {
      Maestro_SetAngle(legIdx, legs[legIdx].servoAngle);
    }

    position.isNew = false;
    EnableInterrupts();

    Maestro_WaitForIdle();
  }
}
