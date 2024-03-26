#include "IMU.h"

#define SAMPLE_RATE 400

static uint16_t IMU_CalcSDR(uint32_t frequency, uint32_t desired);

/**
 * @brief
 * @param SYS_CLK
 * @param position
 */
void IMU_Init(uint32_t SYS_CLK, volatile Position *position) {
  uint8_t whoAmI = 0; //, MAG_whoAmI = 0;
  uint8_t userCtrl = 0;

  ICM20948_Init(SYS_CLK, SAMPLE_RATE, position);

  ICM20948_Write(GYRO_CONFIG_1_ADDR, GYRO_FS_SEL_2000 | GYRO_DLPF_ENABLE); // Configure gyro scale to 2000dps and enable Low-pass filter
  gyroscopeSensitivity.axis.x = gyroscopeSensitivity.axis.y = gyroscopeSensitivity.axis.z = GYRO_2000_SENSITIVITY;
  // Change offset scale from min 250 dps to chosen gyro scale
  gyroscopeOffset.axis.x /= 2000 / 250;
  gyroscopeOffset.axis.y /= 2000 / 250;
  gyroscopeOffset.axis.z /= 2000 / 250;

  ICM20948_Write(ACCEL_CONFIG_ADDR, ACCEL_FS_SEL_8G | ACCEL_DLPF_ENABLE); // Configure accelerometer scale to 8G
  accelerometerSensitivity.axis.x = accelerometerSensitivity.axis.y = accelerometerSensitivity.axis.z = ACCEL_8G_SENSITIVITY;
  // Change offset scale from min 2G to chosen accel scale
  accelerometerOffset.axis.x /= 8 / 2;
  accelerometerOffset.axis.y /= 8 / 2;
  accelerometerOffset.axis.z /= 8 / 2;

  ICM20948_Write(ODR_ALIGN_EN_ADDR, ODR_ALIGN_ENABLE);                                               // Align output data rate
  ICM20948_Write(GYRO_SMPLRT_DIV_ADDR, IMU_CalcSDR(GYRO_MAX_SMPLRT, SAMPLE_RATE) & 0xFF);            // Configure sample rate
  ICM20948_Write(ACCEL_SMPLRT_DIV_1_ADDR, (IMU_CalcSDR(ACCEL_MAX_SMPLRT, SAMPLE_RATE) >> 8) & 0xFF); // Configure (MSB) sample rate
  ICM20948_Write(ACCEL_SMPLRT_DIV_2_ADDR, IMU_CalcSDR(ACCEL_MAX_SMPLRT, SAMPLE_RATE) & 0xFF);        // Configure (LSB) sample rate

  ICM20948_Read(USER_CTRL_ADDR, &userCtrl);
  ICM20948_Write(USER_CTRL_ADDR, (userCtrl | SPI_ENABLE) & ~(DMP_ENABLE | FIFO_ENABLE)); // Enable SPI

  // ICM20948_Mag_Init(); // Enable I2C master for Magnetometer read

  do {
    ICM20948_Read(WHO_AM_I_ADDR, &whoAmI); // Read IMU Identifier
  } while (whoAmI != 0xEA);

  // do {
  //   ICM20948_Mag_Read(MAG_WHO_AM_I, &MAG_whoAmI, 1); // Confirm communication success
  // } while (MAG_whoAmI != 0x09);

  // ICM20948_MagCalibration();       // Run Calibrations
  ICM20948_AccelGyroCalibration(); // Will be empty functions if undesired
}

/**
 * @brief
 * @param
 */
void IMU_Enable(void) {
  // Specify the Interrupt pin is push-pull and is an active high pin (falling edge interrupt)
  // also forces the Interrupt to be cleared for the level to be reset and any Read operation to clear the INT_STATUS
  ICM20948_Write(INT_PIN_CFG_ADDR, INT_READ_CLEAR & ~(INT_ACTIVE_LOW | INT_OPEN_DRAIN | INT_LATCH_MANUAL_CLEAR));
  ICM20948_Write(INT_ENABLE_1_ADDR, RAW_DATA_INT_ENABLE); // Enable Raw Data interrupt

  ICM20948_MadgwickFusion_Init(); // Initialize Fusion Algorithm
  ICM20948_Interrupt_Pin_Init();  // Configure the Interrupt pin
}

/**
 * @brief
 * @param
 */
void IMU_Disable(void) {
  ICM20948_Write(INT_ENABLE_1_ADDR, (uint8_t)~RAW_DATA_INT_ENABLE); // Disable Raw Data interrupt
  ICM20948_Interrupt_Pin_Disable();                                 // Disable the Interrupt pin
}

/**
 * @brief
 * @param frequency
 * @param desired
 * @return
 */
static uint16_t IMU_CalcSDR(uint32_t frequency, uint32_t desired) {
  float SMPLRT = 0.0f;

  if (desired > frequency)
    return 0;

  SMPLRT = ((float)frequency / (float)desired) - 1.0f + 0.5f;

  return (uint16_t)(SMPLRT);
}
