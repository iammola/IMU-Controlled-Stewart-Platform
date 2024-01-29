#ifndef __IMU__
#include <stdint.h>


void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK);

void IMU_Read(REG_ADDRESS REGISTER);

void IMU_Write(REG_ADDRESS REGISTER, uint8_t data);

#endif // __IMU__
