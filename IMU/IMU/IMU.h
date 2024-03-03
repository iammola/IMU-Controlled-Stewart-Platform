#include <stdbool.h>
#include <stdint.h>

#include "Fusion/Fusion.h"

#define INT_BIT      (unsigned)(1 << 6) // (PD6) Interrupt Pin
#define INT_PCTL_M   (unsigned)GPIO_PCTL_PD6_M
#define INT_PRIORITY 1

#define CLI_TXT_BUF 1000
extern char text[CLI_TXT_BUF];

extern volatile float deltaTime;

extern FusionAhrs   ahrs;
extern FusionEuler  euler;
extern FusionOffset offset;

extern FusionVector rawGyroscope;
extern FusionVector rawAccelerometer;
extern FusionVector rawMagnetometer;

extern volatile bool HasNewIMUAngles;

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, FusionVector *gyroSensitivity, FusionVector *accelSensitivity);
