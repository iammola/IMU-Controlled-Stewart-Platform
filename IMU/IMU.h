#ifndef __IMU__
#include <stdint.h>

#include "ICM-20948.h"

static void IMU_Interrupt_Init(void);

static void IMU_Config(void);

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK);

uint8_t IMU_Read(REG_ADDRESS REGISTER);

void IMU_Write(REG_ADDRESS REGISTER, uint8_t data);

#endif // __IMU__
