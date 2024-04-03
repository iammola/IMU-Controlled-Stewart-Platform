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

#define SYS_CLOCK 80e6

static char text[CLI_TXT_BUF] = "";

int main(void) {
  FusionQuaternion quat = {0};
  FusionEuler      euler = {0};

  volatile Position position = {0};

  // 0.582978 0.640293 0.336891 -0.370012 - Accel, Gyro no Z-axis
  // 0.778757 0.489159 -0.332855 0.209075 - Accel, Gyro Z-axis
  // 0.628237 0.014747 -0.585671 -0.512185 - Accel, Gyro, Mag
  const Quaternion IMU_Quat_Offset = QuaternionInverse(0.604001f, 0.622846f, -0.294529f, -0.400927f);

  PLL_Init();              // Initialize the PLL
  FPULazyStackingEnable(); // Enable Floating Point

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  IMU_Init(SYS_CLOCK, &position); // Initialize the IMU
  IMU_Enable();

  while (1) {
    if (!position.isNew)
      continue;

#define Q position.quaternion
#define T position.translation
    Q = QuaternionMultiply(Q, IMU_Quat_Offset);
    // Serial Plot
    // quat.element.w = Q.w;
    // quat.element.x = Q.x;
    // quat.element.y = Q.y;
    // quat.element.z = Q.z;
    // euler = FusionQuaternionToEuler(quat);
    // snprintf(text, CLI_TXT_BUF, "%0.6f %0.6f %0.6f", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

    // AdaFruit_3D_Model_Viewer
    // snprintf(text, CLI_TXT_BUF, "Quaternion: %0.6f,%0.6f,%0.6f,%0.6f", Q.w, Q.x, Q.y, Q.z);

    // Stewart Platform
    snprintf(text, CLI_TXT_BUF, "%0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f", Q.w, Q.x, Q.y, Q.z, T.x, T.y, T.z);
#undef Q
#undef T

    CLI_Write(text);
    CLI_Write("\n");

    position.isNew = false;
  }
}
