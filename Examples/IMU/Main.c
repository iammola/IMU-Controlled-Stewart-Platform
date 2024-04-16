/**
 * @file Main.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stdint.h>
#include <stdio.h>

#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "CLI/CLI.h"
#include "IMU/IMU.h"
#include "UTILS/UTILS.h"

#define SYS_CLOCK           80e6
#define IMU_SAMPLE_RATE     400
#define SAMPLES_BEFORE_PING ((5 * IMU_SAMPLE_RATE) / 100)

void WaitForInterrupt(void);

static char text[CLI_TXT_BUF] = "";

int main(void) {
  volatile Position position = {0};
  const Quaternion  IMU_Quat_Offset = QuaternionConjugate(0.609836638f, 0.529993773f, 0.468245029f, 0.358044267f);

  PLL_Init();              // Initialize the PLL
  FPULazyStackingEnable(); // Enable Floating Point

  CLI_Init(SYS_CLOCK, 921600, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT);

  IMU_Init(SYS_CLOCK, IMU_SAMPLE_RATE, &position); // Initialize the IMU
  IMU_Enable(true);

  while (1) {
    WaitForInterrupt();

    if (position.count < SAMPLES_BEFORE_PING)
      continue;

    position.inUse = true;
    position.quaternion = QuaternionMultiply(position.quaternion, IMU_Quat_Offset, 1.0f);

#define Q position.quaternion
    snprintf(text, CLI_TXT_BUF, "Q: %0.6f %0.6f %0.6f %0.6f\n", Q.w, Q.x, Q.y, Q.z);
    CLI_Write(text);
#undef Q

    position.inUse = false;
    position.count = 0;
  }
}
