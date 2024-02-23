#include <stdint.h>

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

static void IMU_Config(void);
static void IMU_ChangeUserBank(REG_ADDRESS REGISTER);
static void IMU_Read(REG_ADDRESS REGISTER, uint8_t *dest);
static void IMU_Write(REG_ADDRESS REGISTER, uint8_t data);

static void IMU_Mag_ReadWhoAMI(void);
static void IMU_Mag_StartDataRead(void);
static void IMU_Mag_Write(uint8_t MAG_ADDRESS, uint8_t data);

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
#define I2C_MST_ODR_1K  0
#define I2C_MST_ODR_137 3

// GYRO_CONFIG_1
#define GYRO_FS_SEL_1000 0x04
#define GYRO_FS_SEL_1000_SENSITIVITY SENSITIVITY(1000, 11 / 630)

// ACCEL_CONFIG
#define ACCEL_FS_SEL_8G 0x04
// 16 bit ADC from -32,768 to 32,768
#define ACCEL_FS_SEL_8G_SENSITIVITY  SENSITIVITY(8, 9.81)

#define SENSITIVITY(scale, unitRate) ((1 << 15) / (scale * unitRate))

// MAG_CNTL3
#define MAG_RESET 0x01

// MAG_CNTL2
#define MAG_CONT_MODE_4 0x08

// MAG_ST1
#define MAG_DATA_RDY 0x01
