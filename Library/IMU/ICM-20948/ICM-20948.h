/**
 * @file ICM-20948.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief Simple Library to communicate with the ICM-20948 and fuse the sensor
 * data for the absolute orientation of the device using Madgwick's Fusion algorithm
 * in the NWU axis convention
 * @version 0.1
 * @date 2024-03-10
 *
 * @copyright Copyright (c) 2024
 */
#ifndef ICM20948_H
#define ICM20948_H
#include <stdint.h>

#include "Fusion/Fusion.h"
#include "IMU.h"

#define INT_BIT      (unsigned)(1 << 6) // (PD6) Interrupt Pin
#define INT_PCTL_M   (unsigned)GPIO_PCTL_PD6_M
#define INT_PRIORITY 1

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

#define TEMP_DISABLE 0x08

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
#define GYRO_FS_SEL_2000 (3 << 1)
#define GYRO_DLPF_12HZ   (5 << 3)

// ACCEL_CONFIG
#define ACCEL_DLPF_ENABLE 0x01
#define ACCEL_FS_SEL_2G   (0 << 1)
#define ACCEL_FS_SEL_8G   (2 << 1)
#define ACCEL_DLPF_246Hz  (0 << 3)

// INT_ENABLE_1
#define RAW_DATA_INT_ENABLE 0x01

// INT_PIN_CFG
#define INT_ACTIVE_LOW         0x80
#define INT_OPEN_DRAIN         0x40
#define INT_LATCH_MANUAL_CLEAR 0x20
#define INT_READ_CLEAR         0x10

// ODR_ALIGN_EN
#define ODR_ALIGN_ENABLE 0x01

// MAG ADDRESSES
#define MAG_WHO_AM_I 0x01
#define MAG_ST1      0x10
#define MAG_HXL      0x11
#define MAG_ST2      0x18
#define MAG_CNTL2    0x31
#define MAG_CNTL3    0x32

// MAG_CNTL3
#define MAG_RESET 0x01

// MAG_CNTL2
#define MAG_CONT_MODE_4 0x08

// MAG_ST1
#define MAG_DATA_RDY    0x01
#define MAG_DATA_OVRRUN 0x02

#define SENSITIVITY(scale)    (scale / (1 << 15))
#define ACCEL_8G_SENSITIVITY  SENSITIVITY(8.0f)
#define ACCEL_2G_SENSITIVITY  SENSITIVITY(2.0f)
#define MAG_4912_SENSITIVITY  SENSITIVITY(4912.0f)
#define GYRO_250_SENSITIVITY  SENSITIVITY(250.0f)
#define GYRO_1000_SENSITIVITY SENSITIVITY(1000.0f)
#define GYRO_2000_SENSITIVITY SENSITIVITY(2000.0f)

/**
 * @brief Initializes the SysTick, and the SPI module for communication. It resets the device,
 * turns off Low-Power Mode and the Temp sensor, while auto-selecting the best clock and enabling
 * the gyroscope and accelerometer sensors
 * @param SYS_CLK
 */
void ICM20948_Init(uint32_t SYS_CLK);

/**
 * @brief Configures the Pin connected to the INT pin of the ICM-20948 as a Digital GPIO Input,
 * triggering on the Rising Edge event.
 * @param
 */
void ICM20948_Interrupt_Init(void);

/**
 * @brief Reads data from the specified register while selecting the required User Bank
 * @param REGISTER ICM-20948 Register to read from
 * @param dest Pointer Destination to store response from
 */
void ICM20948_Read(REG_ADDRESS REGISTER, uint8_t *dest);

/**
 * @brief Writes data to the specified register while selecting the required User Bank
 * @param REGISTER ICM-20948 Register to write to
 * @param data Data to write to register
 */
void ICM20948_Write(REG_ADDRESS REGISTER, uint8_t data);

/**
 * @brief Initializes the AK09916 magnetometer connected to the ICM-20948 slave I2C bus line
 * It sets the sensor to sample at a 100 Hz rate in Continuous Mode 4
 * @param
 */
void ICM20948_Mag_Init(void);

/**
 * @brief Reads data byte(s) from the specified AK09916 magnetometer address.
 * @param MAG_ADDRESS Register Address to read from
 * @param dest Pointer destination to store read data
 * @param length Number of bytes to read from Address start
 */
void ICM20948_Mag_Read(uint8_t MAG_ADDRESS, uint8_t *dest, uint8_t length);

/**
 * @brief Writes a byte to the specified AK09916 magnetometer address
 * @param MAG_ADDRESS Register Address to write to
 * @param data Data to write to register
 */
void ICM20948_Mag_Write(uint8_t MAG_ADDRESS, uint8_t data);

/**
 * @brief Initializes Madgwick's Complementary Sensor Fusion algorithm for the NWU convention axis
 * with the configured AHRS settings
 * @param
 */
void ICM20948_MadgwickFusion_Init(void);

/**
 * @brief Outputs magnetometer data in the MotionCal format for calibration.
 * @param
 */
void ICM20948_MagCalibration(void);

/**
 * @brief Takes a specific number of samples at the most sensitive scale of the accelerometer or gyroscope at rest
 * and averages them out to find the offset bias
 * @param
 */
void ICM20948_AccelGyroCalibration(void);

// Gyroscope Offset Bias in LSBs for +/-250 dps
extern FusionVector gyroscopeOffset;
// Accelerometer Offset Bias in LSBs for +/-2G
extern FusionVector accelerometerOffset;
// Gyroscope Sensitivity in dps/LSB
extern FusionVector gyroscopeSensitivity;
// Accelerometer Sensitivity in G/LSBs
extern FusionVector accelerometerSensitivity;

extern const REG_ADDRESS WHO_AM_I_ADDR;
extern const REG_ADDRESS USER_CTRL_ADDR;
extern const REG_ADDRESS LP_CONFIG_ADDR;
extern const REG_ADDRESS PWR_MGMT_1_ADDR;
extern const REG_ADDRESS PWR_MGMT_2_ADDR;
extern const REG_ADDRESS INT_PIN_CFG_ADDR;
extern const REG_ADDRESS INT_ENABLE_1_ADDR;
extern const REG_ADDRESS INT_STATUS_1_ADDR;

extern const REG_ADDRESS ACCEL_XOUT_H_ADDR;
extern const REG_ADDRESS ACCEL_XOUT_L_ADDR;
extern const REG_ADDRESS ACCEL_YOUT_H_ADDR;
extern const REG_ADDRESS ACCEL_YOUT_L_ADDR;
extern const REG_ADDRESS ACCEL_ZOUT_H_ADDR;
extern const REG_ADDRESS ACCEL_ZOUT_L_ADDR;
extern const REG_ADDRESS GYRO_XOUT_H_ADDR;
extern const REG_ADDRESS GYRO_XOUT_L_ADDR;
extern const REG_ADDRESS GYRO_YOUT_H_ADDR;
extern const REG_ADDRESS GYRO_YOUT_L_ADDR;
extern const REG_ADDRESS GYRO_ZOUT_H_ADDR;
extern const REG_ADDRESS GYRO_ZOUT_L_ADDR;
extern const REG_ADDRESS EXT_SLV_DATA_0;
extern const REG_ADDRESS EXT_SLV_DATA_23;

extern const REG_ADDRESS USER_BANK_ADDR;

extern const REG_ADDRESS GYRO_SMPLRT_DIV_ADDR;
extern const REG_ADDRESS GYRO_CONFIG_1_ADDR;
extern const REG_ADDRESS ODR_ALIGN_EN_ADDR;
extern const REG_ADDRESS ACCEL_SMPLRT_DIV_1_ADDR;
extern const REG_ADDRESS ACCEL_SMPLRT_DIV_2_ADDR;
extern const REG_ADDRESS ACCEL_CONFIG_ADDR;

extern const REG_ADDRESS I2C_MST_ODR_CONFIG_ADDR;
extern const REG_ADDRESS I2C_MST_CTRL_ADDR;
extern const REG_ADDRESS I2C_SLV_ADDR_ADDR;
extern const REG_ADDRESS I2C_SLV_REG_ADDR;
extern const REG_ADDRESS I2C_SLV_CTRL_ADDR;
extern const REG_ADDRESS I2C_SLV_DO_ADDR;
#endif
