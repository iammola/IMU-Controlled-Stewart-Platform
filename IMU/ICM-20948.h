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

// https://www.mathworks.com/help/supportpkg/raspberrypiio/ref/icm20948-system-object.html#:~:text=%270x69%27%20is%20the%20I2C%20address%20of%20the%20accelerometer%20and%20gyroscope%20of%20ICM%2D20948.%20%270x0C%27%20is%20the%20I2C%20address%20of%20the%20magnetometer%20of%20ICM%2D20948.
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
#define GYRO_FS_SEL_250  0x00
#define GYRO_FS_SEL_500  0x02
#define GYRO_FS_SEL_1000 0x04
#define GYRO_FS_SEL_2000 0x06

// ACCEL_CONFIG
#define ACCEL_FS_SEL_2G  0x00
#define ACCEL_FS_SEL_4G  0x02
#define ACCEL_FS_SEL_8G  0x04
#define ACCEL_FS_SEL_16G 0x06

// MAG_CNTL3
#define MAG_RESET 0x01

// MAG_CNTL2
#define MAG_CONT_MODE_4 0x08

// MAG_ST1
#define MAG_DATA_RDY 0x01
