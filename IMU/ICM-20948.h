#ifndef __ICM_20948__
#include <stdint.h>

#define __ICM_20948__

// PWR_MGMT_1
#define SLEEP_ENABLED 0x40
#define LP_ENABLED 0x20
#define TEMP_ENABLED 0x08
#define CLKSEL_INTERNAL 0x00
#define CLKSEL_AUTO 0x01
#define CLKSEL_STOP 0x07

// PWR_MGMT_2
#define ACCEL_ENABLED 0x38
#define GYRO_ENABLED 0x07

// USER_CTRL
#define DMP_ENABLED 0x80
#define FIFO_ENABLED 0x40
#define SPI_ENABLED 0x10

// GYRO_CONFIG_1
#define GYRO_FS_SEL_250 0x00
#define GYRO_FS_SEL_500 0x02
#define GYRO_FS_SEL_1000 0x04
#define GYRO_FS_SEL_2000 0x06

// ACCEL_CONFIG
#define ACCEL_FS_SEL_2G 0x00
#define ACCEL_FS_SEL_4G 0x02
#define ACCEL_FS_SEL_8G 0x04
#define ACCEL_FS_SEL_16G 0x06

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

REG_ADDRESS USER_CTRL_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x03};
REG_ADDRESS PWR_MGMT_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x06};
REG_ADDRESS PWR_MGMT_2_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x07};
REG_ADDRESS USER_BANK_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x7F};

REG_ADDRESS GYRO_CONFIG_1 = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x01};
REG_ADDRESS ACCEL_CONFIG = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x14};
#endif // __ICM_20948__
