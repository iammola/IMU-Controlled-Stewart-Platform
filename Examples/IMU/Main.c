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

#include "IMU/IMU.h"
#include "UTILS/UTILS.h"
#include "Wireless/Wireless.h"

#define SYS_CLOCK           80e6
#define IMU_SAMPLE_RATE     400
#define SAMPLES_BEFORE_PING ((5 * IMU_SAMPLE_RATE) / 100)

void WaitForInterrupt(void);

int main(void) {
  volatile Position position = {0};
  const Quaternion  IMU_Quat_Offset = QuaternionConjugate(0.575982034f, 0.709781229f, -0.0408605039f, -0.403768331f);

  PLL_Init();              // Initialize the PLL
  FPULazyStackingEnable(); // Enable Floating Point

  Wireless_Init(SYS_CLOCK, false);

  IMU_Init(SYS_CLOCK, IMU_SAMPLE_RATE, &position); // Initialize the IMU
  IMU_Enable();

  while (1) {
    WaitForInterrupt();

    if (position.count < SAMPLES_BEFORE_PING)
      continue;

    position.inUse = true;
    position.quaternion = QuaternionMultiply(position.quaternion, IMU_Quat_Offset, 1.0f);

    Wireless_Transmit(NEW_POSITION, (uint8_t *)&position, NEW_POSITION_LENGTH);

    position.inUse = false;
    position.count = 0;
  }
}
