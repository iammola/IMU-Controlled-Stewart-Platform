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

#include "CLI/CLI.h"
#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "IMU/IMU.h"

#define SYS_CLOCK       80e6
#define IMU_SAMPLE_RATE 400

static char text[CLI_TXT_BUF] = "";

void WaitForInterrupt(void);

int main(void) {
  volatile Position position = {0};
  const Quaternion  IMU_Quat_Offset = QuaternionConjugate(0.604001f, 0.622846f, -0.294529f, -0.400927f);

  PLL_Init();              // Initialize the PLL
  FPULazyStackingEnable(); // Enable Floating Point

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  IMU_Init(SYS_CLOCK, IMU_SAMPLE_RATE, &position); // Initialize the IMU
  IMU_Enable();

  while (1) {
    WaitForInterrupt();

    position.inUse = true;
    position.quaternion = QuaternionMultiply(position.quaternion, IMU_Quat_Offset, 1.0f);
    snprintf(text, CLI_TXT_BUF,
             // "Quaternion: %0.6f,%0.6f,%0.6f,%0.6f\n", // AdaFruit_3D_Model_Viewer
             "%0.6f %0.6f %0.6f %0.6f\n", // Stewart Platform
             position.quaternion.w, position.quaternion.x, position.quaternion.y, position.quaternion.z);
    position.inUse = false;

    CLI_Write(text);
  }
}
