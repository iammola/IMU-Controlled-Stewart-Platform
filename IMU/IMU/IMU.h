#include <stdint.h>
#include <stdbool.h>

#include "Fusion/Fusion.h"

extern FusionEuler euler;
extern bool HasNewIMUAngles;

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK);
