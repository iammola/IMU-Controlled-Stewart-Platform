#include <stdint.h>
#include <stdio.h>

#include "PLL.h"
#include "TivaWareFPU/fpu.h"

#include "CLI/CLI.h"

#include "IMU/IMU.h"
#include "StewartPlatform/StewartPlatform.h"

#define SYS_CLOCK 80e6

int main(void) {
  FusionEuler      euler = {0};
  FusionQuaternion quaternion = {0};

  FusionVector gyroscope = {
      .axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}
  };
  FusionVector gyroscopeOffset = {
      .axis = {.x = 1583.717f, .y = -665.774f, .z = 69.118f}
  };
  FusionVector gyroscopeSensitivity = {
      .axis = {.x = 1.0f, .y = 1.0f, .z = 1.0f}
  };
  FusionMatrix gyroscopeMisalignment = {
      .array = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
  };

  FusionVector accelerometer = {
      .axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}
  };
  FusionVector accelerometerOffset = {
      .axis = {.x = -1344.858f, .y = 276.737f, .z = -17029.416f}
  };
  FusionVector accelerometerSensitivity = {
      .axis = {.x = 1.0f, .y = 1.0f, .z = 1.0f}
  };
  FusionMatrix accelerometerMisalignment = {
      .array = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
  };

  FusionVector magnetometer = {
      .axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}
  };
  FusionMatrix softIronMatrix = {
      .array = {{0.988f, -0.005f, 0.001f}, {-0.005f, 0.997f, 0.006f}, {0.001f, 0.006f, 1.015f}}
  };
  FusionVector hardIronOffset = {
      .axis = {.x = -18.0f, .y = 6.54f, .z = -47.85f}
  };

  FPULazyStackingEnable(); // Enable Floating Point for use especially in Interrupts

  PLL_Init(); // Initialize the PLL

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  // Initialize the IMU with the SYS_CLOCK and to communicate at 1MHz
  IMU_Init(SYS_CLOCK, 1e6, &gyroscopeSensitivity, &gyroscopeOffset, &accelerometerSensitivity, &accelerometerOffset);

  // StewartPlatform_Init(6.2f, 5, 5.08f, 10, 0.2269f, 0.2269f, 0); // Initialize stewart platform with variables from example

  while (1) {
    if (!HasNewIMUAngles)
      continue;

    // Apply calibrations
    gyroscope = FusionCalibrationInertial(rawGyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(rawAccelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(rawMagnetometer, softIronMatrix, hardIronOffset);

    gyroscope = FusionOffsetUpdate(&offset, gyroscope);                         // Update gyroscope offset correction algorithm
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime); // Update gyroscope AHRS algorithm

    quaternion = FusionAhrsGetQuaternion(&ahrs); // Get Quaternions
    euler = FusionQuaternionToEuler(quaternion); // Calculate euler angles

    // AdaFruit_3D_Model_Viewer
    // snprintf(text, CLI_TXT_BUF, "Orientation: %0.4f,%0.4f,%0.4f", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    snprintf(text, CLI_TXT_BUF, "Quaternion: %0.4f,%0.4f,%0.4f,%0.4f", quaternion.element.w, quaternion.element.x, quaternion.element.y,
             quaternion.element.z);

    // Serial Plot
    // snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f ", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);
    // CLI_Write(text);
    // snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f ", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
    // CLI_Write(text);
    // snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f ", magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);
    // CLI_Write(text);
    // snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f ", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

    CLI_Write(text);
    CLI_Write("\n");

    HasNewIMUAngles = false;
  }
}
