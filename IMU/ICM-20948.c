#include "ICM-20948.h"

REG_ADDRESS USER_CTRL_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x03};
REG_ADDRESS PWR_MGMT_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x06};
REG_ADDRESS PWR_MGMT_2_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x07};
REG_ADDRESS USER_BANK_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x7F};
REG_ADDRESS GYRO_CONFIG_1_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x01};
REG_ADDRESS ACCEL_CONFIG_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x14};
REG_ADDRESS INT_ENABLE_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x10};
REG_ADDRESS INT_PIN_CFG_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x0F};
REG_ADDRESS INT_STATUS_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x19};
REG_ADDRESS FIFO_R_W_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x72};
REG_ADDRESS WHO_AM_I_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x00};
REG_ADDRESS GYRO_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x33};
REG_ADDRESS GYRO_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x34};
REG_ADDRESS GYRO_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x35};
REG_ADDRESS GYRO_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x36};
