#include <stdint.h>

#include "Fusion/Fusion.h"
#include "IMU.h"

typedef enum USER_BANK_STRUCT {
  USER_BANK_0 = 0x00,
  USER_BANK_1 = 0x10,
  USER_BANK_2 = 0x20,
  USER_BANK_3 = 0x30,
} USER_BANK;

typedef struct REG_ADDRESS_STRUCT {
  USER_BANK USER_BANK;
  uint8_t   ADDRESS;
} REG_ADDRESS;

// 10.1.1.4. Slave Address (AK09916 data sheet)
// "The slave address of AK09916 is 0Ch"
#define MAG_I2C_ADDRESS 0x0C

// PWR_MGMT_1
#define DEVICE_RESET 0x80
#define SLEEP_ENABLE 0x40
#define LP_ENABLE    0x20

#define TEMP_ENABLE 0x08

#define CLKSEL_INTERNAL 0x00
#define CLKSEL_AUTO     0x01
#define CLKSEL_STOP     0x07

// PWR_MGMT_2
#define ACCEL_DISABLE 0x38
#define GYRO_DISABLE  0x07

// USER_CTRL
#define DMP_ENABLE     0x80
#define FIFO_ENABLE    0x40
#define SPI_ENABLE     0x10
#define I2C_MST_RST    0x02
#define I2C_MST_ENABLE 0x20

// I2C_MST_CLK
#define I2C_MST_CLK_400K 0x07

// LP_CONFIG
#define I2C_MST_ODR 0x40

// I2C_MST_ODR_CONFIG_ADDR
#define I2C_MST_ODR_1K  0x00
#define I2C_MST_ODR_137 0x03

// GYRO_CONFIG_1
#define GYRO_DLPF_ENABLE 0x01
#define GYRO_FS_SEL_250  (0 << 1)
#define GYRO_FS_SEL_1000 (2 << 1)
#define GYRO_DLPF_12HZ   (5 << 3)

// ACCEL_CONFIG
#define ACCEL_DLPF_ENABLE 0x01
#define ACCEL_FS_SEL_2G   (0 << 1)
#define ACCEL_FS_SEL_8G   (2 << 1)
#define ACCEL_DLPF_246Hz  (0 << 3)

// MAG_CNTL3
#define MAG_RESET 0x01

// MAG_CNTL2
#define MAG_CONT_MODE_4 0x08

// MAG_ST1
#define MAG_DATA_RDY 0x01

// INT_ENABLE_1
#define RAW_DATA_INT_ENABLE 0x01

// INT_PIN_CFG
#define INT_ACTIVE_LOW         0x80
#define INT_OPEN_DRAIN         0x40
#define INT_LATCH_MANUAL_CLEAR 0x20
#define INT_READ_CLEAR         0x10

// ODR_ALIGN_EN
#define ODR_ALIGN_ENABLE 0x01