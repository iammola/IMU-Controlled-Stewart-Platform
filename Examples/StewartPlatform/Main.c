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
//
// This file can be used by the Keil uVision configuration wizard to set
// the following system clock configuration values.  Or the value of the
// macros can be directly edited below if not using the uVision configuration
// wizard.
//

//      <o> CTL: Control Method
//              < 0=>  0: ICM-20948 MU
//              < 1=>  1: Joystick
//          <i> Input control method
//
#define CONTROL_METHOD 0

//-------- <<< end of configuration section >>> ------------------------------
#include <stdint.h>
#include <stdio.h>

#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "CLI/CLI.h"

#include "IMU/IMU.h"
#include "Joystick/Joystick.h"
#include "Pololu Maestro/Maestro.h"
#include "Quaternion/Quaternion.h"
#include "StewartPlatform/StewartPlatform.h"

#define SYS_CLOCK 80e6

static char text[CLI_TXT_BUF] = "";

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

int main(void) {
  uint8_t  legIdx = 0;
  Position position = {0};

  PLL_Init();              // Initialize the PLL
  FPULazyStackingEnable(); // Enable Floating Point

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

#if CONTROL_METHOD == 0
  IMU_Init(SYS_CLOCK, &position); // Initialize IMU
  IMU_Enable();
#elif CONTROL_METHOD == 1
  Joystick_Init(SYS_CLOCK, 100, &position); // Initialize Joystick
  Joystick_Enable();
#endif
  Maestro_Init(SYS_CLOCK); // Initialize Maestro Controller
  StewartPlatform_Init();  // Initialize stewart platform

  while (1) {
    WaitForInterrupt();

    if (!position.isNew)
      continue;

    StewartPlatform_Update(position.translation, position.quaternion);

    DisableInterrupts();
    for (legIdx = 0; legIdx < Maestro_Channels; legIdx++) {
      Maestro_SetAngle(legIdx, legs[legIdx].servoAngle);
    }
    Maestro_WaitForIdle();
    EnableInterrupts();

    position.isNew = false;
  }
}
