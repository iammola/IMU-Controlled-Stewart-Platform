#include <stdint.h>
#include <stdio.h>

#include "CLI/CLI.h"
#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "IMU/IMU.h"

#define SYS_CLOCK 80e6

static char text[CLI_TXT_BUF] = "";

int main(void) {
  FPULazyStackingEnable(); // Enable Floating Point for use especially in Interrupts

  PLL_Init(); // Initialize the PLL

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  IMU_Init(SYS_CLOCK); // Initialize the IMU

  while (1) {
    if (!HasNewIMUAngles)
      continue;

    // AdaFruit_3D_Model_Viewer
    // snprintf(text, CLI_TXT_BUF, "Quaternion: %0.4f,%0.4f,%0.4f,%0.4f", quaternion.element.w, quaternion.element.x, quaternion.element.y,
    //          quaternion.element.z);

    // Serial Plot
    // snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f ", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);
    // CLI_Write(text);
    // snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f ", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
    // CLI_Write(text);
    // snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f ", magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);

    // Stewart Platform
    snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f %0.4f", quaternion.element.w, quaternion.element.x, quaternion.element.y, quaternion.element.z);

    CLI_Write(text);
    CLI_Write("\n");

    HasNewIMUAngles = false;
  }
}
