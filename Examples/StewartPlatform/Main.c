#include <stdint.h>
#include <stdio.h>

#include "FPU/fpu.h"
#include "PLL.h"

#include "CLI/CLI.h"

#include "IMU/IMU.h"
#include "Joystick/Joystick.h"
#include "Quaternion/Quaternion.h"
#include "StewartPlatform/StewartPlatform.h"

#define SYS_CLOCK 80e6

static char                text[CLI_TXT_BUF] = "";
static const StewartCoords translation = {0.0f};

void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

int main(void) {
  Quaternion stewartQuaternion = {0.0f};
  FPULazyStackingEnable(); // Enable Floating Point for use especially in Interrupts

  PLL_Init(); // Initialize the PLL

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  IMU_Init(SYS_CLOCK);                          // Initialize IMU
  StewartPlatform_Init(0.0f, 0.0f, 0.0f, 0.0f); // Initialize stewart platform

  while (1) {
    WaitForInterrupt();

    if (HasNewIMUAngles) {
      stewartQuaternion.w = quaternion.element.w;
      stewartQuaternion.x = quaternion.element.x;
      stewartQuaternion.y = quaternion.element.y;
      stewartQuaternion.z = quaternion.element.z;

      HasNewIMUAngles = false;
    } else if (HasNewJoystickCoords) {
      stewartQuaternion = normalizeQuaternion(-13.0f, -cosf(coords.angle), sinf(coords.angle), 0.0f);
      HasNewJoystickCoords = false;
    } else
      continue;

    StewartPlatform_Update(translation, stewartQuaternion);

    snprintf(text, CLI_TXT_BUF, "%0.6f %0.6f %0.6f %0.6f %0.6f %0.6f", legs[0].servoAngle, legs[1].servoAngle, legs[2].servoAngle, legs[3].servoAngle,
             legs[4].servoAngle, legs[5].servoAngle);

    DisableInterrupts();
    CLI_Write(text);
    CLI_Write("\n");
    EnableInterrupts();
  }
}
