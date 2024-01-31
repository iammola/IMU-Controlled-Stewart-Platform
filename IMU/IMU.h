#ifndef __IMU__

#include <stdint.h>

#include "ICM-20948.h"

#define __IMU__

typedef void (*DELAY_FUNC)(uint32_t timeInMs);

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, DELAY_FUNC DELAY);

uint16_t IMU_Read(REG_ADDRESS REGISTER);

void IMU_Write(REG_ADDRESS REGISTER, uint8_t data);

#endif // __IMU__
