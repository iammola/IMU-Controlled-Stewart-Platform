#include <stdbool.h>
#include <stdint.h>

#include "ICM-20948/ICM-20948.h"

extern volatile FusionQuaternion quaternion;
extern volatile bool             HasNewIMUAngles;

void IMU_Init(uint32_t SYS_CLK);
