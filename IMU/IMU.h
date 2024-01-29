#ifndef __IMU__
#include <stdint.h>

#define __IMU__

typedef enum USER_BANK_STRUCT
{
  USER_BANK_0 = 0x00,
  USER_BANK_1 = 0x10,
  USER_BANK_2 = 0x20,
  USER_BANK_3 = 0x30,
} USER_BANK;

typedef struct REG_ADDRESS_STRUCT
{
  USER_BANK USER_BANK;
  uint8_t ADDRESS;
} REG_ADDRESS;

REG_ADDRESS USER_CTRL_ADDR = {
    .USER_BANK = USER_BANK_0,
    .ADDRESS = 0x03,
};

REG_ADDRESS PWR_MGMT_1_ADDR = {
    .USER_BANK = USER_BANK_0,
    .ADDRESS = 0x06,
};

REG_ADDRESS PWR_MGMT_2_ADDR = {
    .USER_BANK = USER_BANK_0,
    .ADDRESS = 0x07,
};

REG_ADDRESS USER_BANK_ADDR = {
    .USER_BANK = USER_BANK_0,
    .ADDRESS = 0x7F,
};

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK);
void IMU_Read(REG_ADDRESS REGISTER);
void IMU_Write(REG_ADDRESS REGISTER, uint8_t data);
#endif // __IMU__
