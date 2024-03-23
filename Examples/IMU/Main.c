#include <stdint.h>
#include <stdio.h>

#include "CLI/CLI.h"
#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "IMU/IMU.h"

#define SYS_CLOCK 80e6

static char text[CLI_TXT_BUF] = "";

int main(void) {
  volatile Position position = {0};

  PLL_Init();              // Initialize the PLL
  FPULazyStackingEnable(); // Enable Floating Point

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  IMU_Init(SYS_CLOCK, &position); // Initialize the IMU
  IMU_Enable();

  while (1) {
    if (!position.isNew)
      continue;

#define Q position.quaternion
#define T position.quaternion
    // AdaFruit_3D_Model_Viewer
    snprintf(text, CLI_TXT_BUF, "Quaternion: %0.4f,%0.4f,%0.4f,%0.4f", Q.w, Q.x, Q.y, Q.z);
    // Stewart Platform
    snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f", Q.w, Q.x, Q.y, Q.z, T.x, T.y, T.z);
#undef T
#undef Q

    CLI_Write(text);
    CLI_Write("\n");

    position.isNew = false;
  }
}
