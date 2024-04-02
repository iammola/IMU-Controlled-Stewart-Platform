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
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------

//      <o> CAL: Calibration
//              < 0=>  0: Off
//              < 1=>  1: Accelerometer
//              < 2=>  2: Accelerometer Test
//              < 3=>  3: Gyroscope
//              < 4=>  4: Gyroscope Test
//              < 5=>  5: Magnetometer
//              < 6=>  6: Magnetometer Test
//          <i> The calibration state to put the IMU in
//
#define CALIBRATION_MODE 0

//-------- <<< end of configuration section >>> ------------------------------

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "CLI/CLI.h"
#include "SSI/SSI2.h"
#include "SysTick/SysTick.h"

#include "Fusion/Fusion.h"
#include "ICM-20948.h"

#include "tm4c123gh6pm.h"

#define READ(addr)        (uint16_t)(0x80FF | (addr << 8))
#define WRITE(addr, data) (uint16_t)(0x7FFF & ((addr << 8) | data))

#define SSI_SPEED 1e6

#define INT_BIT      (unsigned)(1 << 3) // (PB3) Interrupt Pin
#define INT_PCTL_M   (unsigned)GPIO_PCTL_PB3_M
#define INT_PRIORITY 2

static USER_BANK LastUserBank = 0xFF;

void GPIOB_Handler(void);

static void ICM20948_ChangeUserBank(REG_ADDRESS REGISTER);

static bool ICM20948_GetMagReadings(FusionVector *dest);
static void ICM20948_GetRawGyroReadings(FusionVector *dest);
static void ICM20948_GetRawAccelReadings(FusionVector *dest);

static FusionAhrs   ahrs = {0};
static FusionOffset offset = {0};

static volatile Position *__position;

static uint32_t __sysClock;
static uint32_t __sampleRate;

static volatile float    deltaTime = 0.0f;
static volatile uint32_t lastTimestamp = 0;

static FusionVector gyroscope = FUSION_VECTOR_ZERO;
static FusionVector rawGyroscope = FUSION_VECTOR_ZERO;

static FusionVector accelerometer = FUSION_VECTOR_ZERO;
static FusionVector rawAccelerometer = FUSION_VECTOR_ZERO;

static FusionVector magnetometer = FUSION_VECTOR_ZERO;
static FusionVector rawMagnetometer = FUSION_VECTOR_ZERO;

static FusionMatrix softIronMatrix = {
    .array = {{0.988f, -0.005f, 0.001f}, {-0.005f, 0.997f, 0.006f}, {0.001f, 0.006f, 1.015f}}
};
static FusionVector hardIronOffset = {
    .axis = {.x = -18.0f, .y = 6.54f, .z = -47.85f}
};

FusionVector gyroscopeSensitivity = FUSION_VECTOR_ONES;
FusionVector accelerometerSensitivity = FUSION_VECTOR_ONES;

FusionVector gyroscopeOffset = {
    .axis = {.x = 1583.04468f, .y = -668.109802f, .z = 71.4820862f}
};
FusionVector accelerometerOffset = {
    .axis = {.x = -6168.63965f, .y = -4822.40723f, .z = 13846.6348f}
};

/**
 * @brief ICM-20948 Interrupt Handler
 * @param
 */
void GPIOB_Handler(void) {
  uint8_t           intStatus = 0;
  volatile uint32_t timestamp = ST_CURRENT_R;

  GPIO_PORTB_ICR_R |= INT_BIT; // Clear Interrupt

  ICM20948_Read(INT_STATUS_1_ADDR, &intStatus);
  if (!intStatus)
    return;

  // Read sensor data
  ICM20948_GetRawGyroReadings(&rawGyroscope);
  ICM20948_GetRawAccelReadings(&rawAccelerometer);
  ICM20948_GetMagReadings(&rawMagnetometer);

  if (lastTimestamp == 0)
    deltaTime = 0.0f; // Start of process
  else if (lastTimestamp < timestamp)
    deltaTime = (float)(ST_RELOAD_R - (timestamp - lastTimestamp)); // Restarted at max reload value, so have to find ticks in reverse
  else
    deltaTime = (float)(lastTimestamp - timestamp); // Ticks since last interrupt

  deltaTime /= (float)(__sysClock); // Calculate delta time (in seconds) to account for gyroscope sample clock error
  lastTimestamp = timestamp;        // Update timestamp tracker

  // Apply calibrations
  gyroscope = FusionCalibrationInertial(rawGyroscope, FUSION_IDENTITY_MATRIX, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(rawAccelerometer, FUSION_IDENTITY_MATRIX, accelerometerSensitivity, accelerometerOffset);
  magnetometer = FusionCalibrationMagnetic(rawMagnetometer, softIronMatrix, hardIronOffset);

  gyroscope = FusionOffsetUpdate(&offset, gyroscope); // Update gyroscope offset correction algorithm

  // FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime); // Update gyroscope AHRS algorithm without magnetometer
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime); // Update gyroscope AHRS algorithm

  __position->quaternion.w = ahrs.quaternion.element.w;
  __position->quaternion.x = ahrs.quaternion.element.x; // Hard-fault here when building with -O0
  __position->quaternion.y = ahrs.quaternion.element.y;
  __position->quaternion.z = ahrs.quaternion.element.z;
  __position->translation.x = 0.0f;
  __position->translation.y = 0.0f;
  __position->translation.z = 0.0f;
  __position->isNew = true;

  // Serial Plot
  // snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f ", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);
  // CLI_Write(text);
  // snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f ", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
  // CLI_Write(text);
  // snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f ", magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);
  // CLI_Write(text);
  // CLI_Write("\n");
}

/**
 * @brief Change the active bank to access the register data
 * @param REGISTER
 */
static void ICM20948_ChangeUserBank(REG_ADDRESS REGISTER) {
  uint16_t byte = WRITE(USER_BANK_ADDR.ADDRESS, REGISTER.USER_BANK);

  if (REGISTER.ADDRESS == USER_BANK_ADDR.ADDRESS)
    return;

  SSI2_StartTransmission();
  SSI2_Write(&byte, 1);
  SSI2_EndTransmission();

  LastUserBank = REGISTER.USER_BANK;
}

/**
 * @brief Get the raw accelerometer vector in LSBs for the NWU co-ordinate
 * @param dest
 */
static void ICM20948_GetRawAccelReadings(FusionVector *dest) {
  uint8_t accelXH = 0;
  uint8_t accelXL = 0;
  uint8_t accelYH = 0;
  uint8_t accelYL = 0;
  uint8_t accelZH = 0;
  uint8_t accelZL = 0;

  ICM20948_Read(ACCEL_XOUT_H_ADDR, &accelXH);
  ICM20948_Read(ACCEL_XOUT_L_ADDR, &accelXL);
  ICM20948_Read(ACCEL_YOUT_H_ADDR, &accelYH);
  ICM20948_Read(ACCEL_YOUT_L_ADDR, &accelYL);
  ICM20948_Read(ACCEL_ZOUT_H_ADDR, &accelZH);
  ICM20948_Read(ACCEL_ZOUT_L_ADDR, &accelZL);

  dest->axis.x = (int16_t)((accelXH << 8) | accelXL) * -1;
  dest->axis.y = (int16_t)((accelYH << 8) | accelYL) * -1;
  dest->axis.z = (int16_t)((accelZH << 8) | accelZL);
}

/**
 * @brief Get the raw gyroscope vector in LSBs for the NWU co-ordinate
 * @param dest
 */
static void ICM20948_GetRawGyroReadings(FusionVector *dest) {
  uint8_t gyroXH = 0;
  uint8_t gyroXL = 0;
  uint8_t gyroYH = 0;
  uint8_t gyroYL = 0;
  uint8_t gyroZH = 0;
  uint8_t gyroZL = 0;

  ICM20948_Read(GYRO_XOUT_H_ADDR, &gyroXH);
  ICM20948_Read(GYRO_XOUT_L_ADDR, &gyroXL);
  ICM20948_Read(GYRO_YOUT_H_ADDR, &gyroYH);
  ICM20948_Read(GYRO_YOUT_L_ADDR, &gyroYL);
  ICM20948_Read(GYRO_ZOUT_H_ADDR, &gyroZH);
  ICM20948_Read(GYRO_ZOUT_L_ADDR, &gyroZL);

  dest->axis.x = (int16_t)((gyroXH << 8) | gyroXL) * -1;
  dest->axis.y = (int16_t)((gyroYH << 8) | gyroYL) * -1;
  dest->axis.z = (int16_t)((gyroZH << 8) | gyroZL);
}

/**
 * @brief Get the magnetometer vector in uT for the NWU co-ordinate
 * @param dest
 * @return true - if new magnetometer data was read
 */
static bool ICM20948_GetMagReadings(FusionVector *dest) {
  uint8_t ST1 = 0;
  uint8_t magCoords[8] = {0};

  ICM20948_Mag_Read(MAG_ST1, &ST1, 1); // Get the status

  if (ST1 & MAG_DATA_RDY) {
    SysTick_WaitCustom(10, -6);
    ICM20948_Mag_Read(MAG_HXL, magCoords, 8); // Get the X,Y,Z bytes data and ST2 required for read end

    dest->axis.x = (int16_t)((magCoords[1] << 8) | magCoords[0]) * MAG_4912_SENSITIVITY * -1;
    dest->axis.y = (int16_t)((magCoords[3] << 8) | magCoords[2]) * MAG_4912_SENSITIVITY;
    dest->axis.z = (int16_t)((magCoords[5] << 8) | magCoords[4]) * MAG_4912_SENSITIVITY * -1;

    return true;
  } else if (ST1 & MAG_DATA_OVRRUN) {
    ICM20948_Mag_Read(MAG_ST2, &ST1, 1); // Read ST2
  }

  return false;

  // magCoords[7] = STATUS_2
}

/**
 * @brief Initializes the SysTick, and the SPI module for communication. It resets the device,
 * turns off Low-Power Mode and the Temp sensor, while auto-selecting the best clock and enabling
 * the gyroscope and accelerometer sensors
 * @param SYS_CLK
 * @param SAMPLE_RATE
 * @param position Struct for IMU location updates
 */
void ICM20948_Init(uint32_t SYS_CLK, uint32_t SAMPLE_RATE, volatile Position *position) {
  if (position == NULL || SYS_CLK == NULL || SAMPLE_RATE == NULL) {
    while (1)
      ;
  }

  __position = position;
  __sysClock = SYS_CLK;
  __sampleRate = SAMPLE_RATE;

  SysTick_Init(); // Initialize SysTick

  SSI2_Init(__sysClock, SSI_SPEED, SSI_MODE3, SSI_DATA_16); // Initialize the SPI pins

  ICM20948_Write(PWR_MGMT_1_ADDR, DEVICE_RESET | SLEEP_ENABLE); // Reset the device
  SysTick_WaitCustom(101, -3);                                  // Wait atleast 100ms after device reset

  ICM20948_Write(PWR_MGMT_1_ADDR, CLKSEL_AUTO | TEMP_DISABLE); // Disables Sleep, Low-Power Mode and Temp Sensor. Auto selects clk
  SysTick_WaitCustom(40, -3);                                  // Wait atleast 35ms after waking from sleep

  ICM20948_Write(PWR_MGMT_2_ADDR, ACCEL_ENABLE(1, 1, 1) | GYRO_ENABLE(1, 1, 1)); // Enable the Accelerometer and Gyroscope without z-axis
  SysTick_WaitCustom(40, -3);                                                    // Wait atleast 35ms after enabling accel and gyro
}

/**
 * @brief Configures the Pin connected to the INT pin of the ICM-20948 as a Digital GPIO Input,
 * triggering on the Rising Edge event.
 * @param
 */
void ICM20948_Interrupt_Pin_Init(void) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // Enable Port D clock

  GPIO_PORTB_AFSEL_R &= ~INT_BIT;   // Disable Alternate Functions on CS pin
  GPIO_PORTB_PCTL_R &= ~INT_PCTL_M; // Disable Peripheral functions on PD6
  GPIO_PORTB_DIR_R &= ~INT_BIT;     // Configure INT bit as input
  GPIO_PORTB_DEN_R |= INT_BIT;      // Enable Digital Mode on pins
  GPIO_PORTB_AMSEL_R &= ~INT_BIT;   // Disable Analog Mode on pins
  GPIO_PORTB_IM_R &= ~INT_BIT;      // Disable the INT pin interrupt
  GPIO_PORTB_IS_R &= ~INT_BIT;      // Configure for Edge-Detect interrupts
  GPIO_PORTB_IBE_R &= ~INT_BIT;     // Only listen on one edge event
  GPIO_PORTB_IEV_R |= INT_BIT;      // Trigger interrupt on rising edge

  NVIC_EN0_R |= NVIC_EN0_INT1;                                                                    // Enable Port B's Interrupt Handler
  NVIC_PRI0_R = (NVIC_PRI0_R & (unsigned)~NVIC_PRI0_INT1_M) | (INT_PRIORITY << NVIC_PRI0_INT1_S); // Configure Port B's priority
  GPIO_PORTB_ICR_R |= INT_BIT;                                                                    // Clear the INT pin's interrupt
  GPIO_PORTB_IM_R |= INT_BIT;                                                                     // Allow the INT pin interrupt to be detected
}

/**
 * @brief Disables the Pin connected to the INT pin of the ICM-20948 from triggering an interrupt
 * @param
 */
void ICM20948_Interrupt_Pin_Disable(void) {
  GPIO_PORTB_IM_R &= ~INT_BIT; // Disable the INT pin interrupt
}

/**
 * @brief Reads data from the specified register while selecting the required User Bank
 * @param REGISTER ICM-20948 Register to read from
 * @param dest Pointer Destination to store response from
 */
void ICM20948_Read(REG_ADDRESS REGISTER, uint8_t *dest) {
  uint16_t response;

  // Change User Bank if Needed
  if (LastUserBank != REGISTER.USER_BANK)
    ICM20948_ChangeUserBank(REGISTER);

  SSI2_StartTransmission();
  SSI2_Read(READ(REGISTER.ADDRESS), &response, 1);
  SSI2_EndTransmission();

  *dest = response & 0xFF;
}

/**
 * @brief Writes data to the specified register while selecting the required User Bank
 * @param REGISTER ICM-20948 Register to write to
 * @param data Data to write to register
 */
void ICM20948_Write(REG_ADDRESS REGISTER, uint8_t data) {
  uint16_t byte = WRITE(REGISTER.ADDRESS, data);

  // Change User Bank if Needed
  if (LastUserBank != REGISTER.USER_BANK)
    ICM20948_ChangeUserBank(REGISTER);

  SSI2_StartTransmission();
  SSI2_Write(&byte, 1);
  SSI2_EndTransmission();
}

/**
 * @brief Initializes the AK09916 magnetometer connected to the ICM-20948 slave I2C bus line
 * It sets the sensor to sample at a 100 Hz rate in Continuous Mode 4
 * @param
 */
void ICM20948_Mag_Init(void) {
  uint8_t tmp = 0;
  uint8_t userCtrl = 0x00;
  ICM20948_Read(USER_CTRL_ADDR, &userCtrl); // Get current user ctrl settings

  ICM20948_Write(USER_CTRL_ADDR, userCtrl | I2C_MST_RST); // Reset I2C master
  SysTick_WaitCustom(100, -3);                            // wait for 100ms after I2C master reset

  ICM20948_Write(USER_CTRL_ADDR, userCtrl | I2C_MST_ENABLE); // enable I2C master
  ICM20948_Write(I2C_MST_CTRL_ADDR, I2C_MST_CLK_400K);       // Use Mast CLK = 345.60kHz, Duty Cycle = 46.67%
  ICM20948_Write(LP_CONFIG_ADDR, I2C_MST_ODR);               // Use I2C_MST_ODR_CONFIG_ADDR for mag sampling rate
  ICM20948_Write(I2C_MST_ODR_CONFIG_ADDR, I2C_MST_ODR_137);  // Use 137Hz rate on magnetometer

  ICM20948_Mag_Write(MAG_CNTL3, MAG_RESET); // reset mag
  SysTick_WaitCustom(105, -6);              // Wait at least 100us

  do {                                              // Poll until in Continuous Mode 4
    ICM20948_Mag_Write(MAG_CNTL2, MAG_CONT_MODE_4); // Use 100 Hz sample rate
    SysTick_WaitCustom(105, -6);                    // Wait at least 100us

    ICM20948_Mag_Read(MAG_CNTL2, &tmp, 1);
  } while (tmp != MAG_CONT_MODE_4);
}

/**
 * @brief Reads data byte(s) from the specified AK09916 magnetometer address.
 * @param MAG_ADDRESS Register Address to read from
 * @param dest Pointer destination to store read data
 * @param length Number of bytes to read from Address start
 */
void ICM20948_Mag_Read(uint8_t MAG_ADDRESS, uint8_t *dest, uint8_t length) {
  uint8_t     dataIdx = 0;
  REG_ADDRESS EXT_REG = EXT_SLV_DATA_0;

  // Max number of registers the external data can be stored into
  if (length > (EXT_SLV_DATA_23.ADDRESS - EXT_SLV_DATA_0.ADDRESS)) {
    return;
  }

  ICM20948_Write(I2C_SLV_ADDR_ADDR, 0x80 | MAG_I2C_ADDRESS); // Read op and set Mag I2C address
  ICM20948_Write(I2C_SLV_REG_ADDR, MAG_ADDRESS);             // Set Magnetometer to start read from desired register
  ICM20948_Write(I2C_SLV_CTRL_ADDR, 0x80 | length);          // Allow transmission for x bytes of data

  SysTick_WaitCustom(10, -6); // Wait 10 us

  for (dataIdx = 0; dataIdx < length; dataIdx++) {
    ICM20948_Read(EXT_REG, dest); // Read data
    EXT_REG.ADDRESS++;
    dest++;
  }
}

/**
 * @brief Writes a byte to the specified AK09916 magnetometer address
 * @param MAG_ADDRESS Register Address to write to
 * @param data Data to write to register
 */
void ICM20948_Mag_Write(uint8_t MAG_ADDRESS, uint8_t data) {
  ICM20948_Write(I2C_SLV_ADDR_ADDR, 0x7F & MAG_I2C_ADDRESS); // Write op and set Mag I2C address
  ICM20948_Write(I2C_SLV_REG_ADDR, MAG_ADDRESS);             // Set Magnetometer Address to read
  ICM20948_Write(I2C_SLV_DO_ADDR, data);                     // Set data to write
  ICM20948_Write(I2C_SLV_CTRL_ADDR, 0x80 | 0x01);            // Enable 1-byte data write
}

/**
 * @brief Initializes Madgwick's Complementary Sensor Fusion algorithm for the NWU convention axis
 * with the configured AHRS settings
 * @param
 */
void ICM20948_MadgwickFusion_Init(void) {
  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu,
      .gain = 10.0f,
      .gyroscopeRange = 2000.0f, // gyroscope range in dps
      .accelerationRejection = 10.0f,
      .magneticRejection = 0,
      .recoveryTriggerPeriod = 5 * __sampleRate, /* 5 seconds */
  };

  FusionOffsetInitialise(&offset, __sampleRate);
  FusionAhrsInitialise(&ahrs);

  FusionAhrsSetSettings(&ahrs, &settings);
}

/**
 * @brief Outputs magnetometer data in the MotionCal format for calibration.
 * @param
 */
void ICM20948_MagCalibration(void) {
#if (CALIBRATION_MODE == 5) || (CALIBRATION_MODE == 6)
  static char text[CLI_TXT_BUF] = "";

  FusionVector magSample = FUSION_VECTOR_ZERO;
  FusionVector sampleHardIronOffset = FUSION_VECTOR_ZERO;
  FusionMatrix sampleSoftIronMatrix = FUSION_IDENTITY_MATRIX;

  uint8_t calibrationDataIdx = 0;
  uint8_t calibrationData[68] = {0};

  float magField = 0.0f;
  float offsets[10] = {0};

  CLI_Init(__sysClock, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  while (1) {
    if (!ICM20948_GetMagReadings(&magSample))
      continue;
#if CALIBRATION_MODE == 6
    magSample = FusionCalibrationMagnetic(magSample, sampleSoftIronMatrix, sampleHardIronOffset);
#endif

    magSample.axis.x *= -1.0f;
    magSample.axis.y *= -1.0f;
    magSample.axis.z *= -1.0f;

#define M magSample.axis
    // Suggested to multiply by 10 for "Good Integer Value"
    snprintf(text, CLI_TXT_BUF, "Raw:0,0,0,0,0,0,%d,%d,%d\n\r", (int)(M.x * 10), (int)(M.y * 10), (int)(M.z * 10));
    CLI_Write(text);
#undef M

    if (!(UART0_FR_R & UART_FR_RXFE)) {
      for (calibrationDataIdx = 0; calibrationDataIdx < 68; calibrationDataIdx++) {
        calibrationData[calibrationDataIdx] = CLI_Read();
      }

      // First two bytes must be 117 (u) and 84 (T)... Micro Tesla?? lol
      if (calibrationData[0] == 117 && calibrationData[1] == 84) {
        // Don't care about the first 26 bytes (4 bytes per float number per axes, 3 axes, 2 sensors - Gyro & Accel )
        memcpy(offsets, calibrationData + 26, 10 * 4);

        sampleHardIronOffset.axis.x = offsets[0];
        sampleHardIronOffset.axis.y = offsets[1];
        sampleHardIronOffset.axis.z = offsets[2];

        magField = offsets[3];

        // Diagonal of 3x3
        sampleSoftIronMatrix.element.xx = offsets[4];
        sampleSoftIronMatrix.element.yy = offsets[5];
        sampleSoftIronMatrix.element.zz = offsets[6];

        sampleSoftIronMatrix.element.xy = sampleSoftIronMatrix.element.yx = offsets[7];
        sampleSoftIronMatrix.element.xz = sampleSoftIronMatrix.element.zx = offsets[8];
        sampleSoftIronMatrix.element.yz = sampleSoftIronMatrix.element.zy = offsets[9];

        while (1)
          ;
      }
    }
  }
#endif
}

/**
 * @brief Takes a specific number of samples at the most sensitive scale of the accelerometer or gyroscope at rest
 * and averages them out to find the offset bias
 * @param
 */
void ICM20948_AccelGyroCalibration(void) {
#if (CALIBRATION_MODE > 0) && (CALIBRATION_MODE < 5)
  static char text[CLI_TXT_BUF] = "";

  uint8_t       rawDataInt = 0;
  uint32_t      SAMPLES_LEFT = 0;
  const int32_t SAMPLES_COUNT = 1e4;

  FusionVector sample = FUSION_VECTOR_ZERO;
  FusionVector totalSamples = FUSION_VECTOR_ZERO;
  FusionVector sampleOffset = FUSION_VECTOR_ZERO;
  FusionVector sampleSensitivity = FUSION_VECTOR_ONES;
  FusionMatrix sampleMisalignment = FUSION_IDENTITY_MATRIX;

#if (CALIBRATION_MODE == 3) || (CALIBRATION_MODE == 1)
  // Reset offset if not in test mode, for new calibration values
  sampleOffset.axis.x = sampleOffset.axis.y = sampleOffset.axis.z = 0;
#endif

#if (CALIBRATION_MODE == 3) || (CALIBRATION_MODE == 4)
  ICM20948_Write(GYRO_CONFIG_1_ADDR, GYRO_DLPF_12HZ | GYRO_FS_SEL_250 | GYRO_DLPF_ENABLE); // Most sensitive scale 250dps, 200Hz BW
  sampleSensitivity.axis.x = sampleSensitivity.axis.y = sampleSensitivity.axis.z = GYRO_250_SENSITIVITY;
#elif (CALIBRATION_MODE == 1) || (CALIBRATION_MODE == 2)
  ICM20948_Write(ACCEL_CONFIG_ADDR, ACCEL_DLPF_246Hz | ACCEL_FS_SEL_2G | ACCEL_DLPF_ENABLE); // Most sensitive range 2G full scale, 246Hz BW
  sampleSensitivity.axis.x = sampleSensitivity.axis.y = sampleSensitivity.axis.z = ACCEL_2G_SENSITIVITY;
#endif

  for (SAMPLES_LEFT = SAMPLES_COUNT; SAMPLES_LEFT > 0; SAMPLES_LEFT--)
  // while (1)
  {
#if (CALIBRATION_MODE == 3) || (CALIBRATION_MODE == 4)
    ICM20948_GetRawGyroReadings(&sample);
#elif (CALIBRATION_MODE == 1) || (CALIBRATION_MODE == 2)
    ICM20948_GetRawAccelReadings(&sample);
#endif
    sample = FusionCalibrationInertial(sample, sampleMisalignment, sampleSensitivity, sampleOffset);

    // This will be in the unit from the sensitivity
    totalSamples.axis.x += sample.axis.x;
    totalSamples.axis.y += sample.axis.y;
    totalSamples.axis.z += sample.axis.z;

    snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f\n", sample.axis.x, sample.axis.y, sample.axis.z);
    CLI_Write(text);

    while (1) {
      ICM20948_Read(INT_STATUS_1_ADDR, &rawDataInt); // Read int status reg

      if (rawDataInt & RAW_DATA_RDY) { // Wait for new data
        rawDataInt &= ~RAW_DATA_RDY;
        break;
      }
    }
  }

  // Normalize back to LSBs
  totalSamples.axis.x /= (SAMPLES_COUNT * sampleSensitivity.axis.x);
  totalSamples.axis.y /= (SAMPLES_COUNT * sampleSensitivity.axis.y);
  totalSamples.axis.z /= (SAMPLES_COUNT * sampleSensitivity.axis.z);

  snprintf(text, CLI_TXT_BUF, "Measurement finished. X = %0.4f Y = %0.4f Z = %0.4f\n", totalSamples.axis.x, totalSamples.axis.y, totalSamples.axis.z);
  CLI_Write(text);

  while (1)
    ;
#endif
}

const REG_ADDRESS WHO_AM_I_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x00};
const REG_ADDRESS USER_CTRL_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x03};
const REG_ADDRESS LP_CONFIG_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x05};
const REG_ADDRESS PWR_MGMT_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x06};
const REG_ADDRESS PWR_MGMT_2_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x07};
const REG_ADDRESS INT_PIN_CFG_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x0F};
const REG_ADDRESS INT_ENABLE_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x11};
const REG_ADDRESS INT_STATUS_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x1A};

const REG_ADDRESS ACCEL_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2D};
const REG_ADDRESS ACCEL_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2E};
const REG_ADDRESS ACCEL_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2F};
const REG_ADDRESS ACCEL_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x30};
const REG_ADDRESS ACCEL_ZOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x31};
const REG_ADDRESS ACCEL_ZOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x32};
const REG_ADDRESS GYRO_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x33};
const REG_ADDRESS GYRO_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x34};
const REG_ADDRESS GYRO_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x35};
const REG_ADDRESS GYRO_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x36};
const REG_ADDRESS GYRO_ZOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x37};
const REG_ADDRESS GYRO_ZOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x38};
const REG_ADDRESS EXT_SLV_DATA_0 = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x3B};
const REG_ADDRESS EXT_SLV_DATA_23 = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x52};

const REG_ADDRESS USER_BANK_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x7F};

const REG_ADDRESS GYRO_SMPLRT_DIV_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x00};
const REG_ADDRESS GYRO_CONFIG_1_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x01};
const REG_ADDRESS ODR_ALIGN_EN_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x09};
const REG_ADDRESS ACCEL_SMPLRT_DIV_1_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x10};
const REG_ADDRESS ACCEL_SMPLRT_DIV_2_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x11};
const REG_ADDRESS ACCEL_CONFIG_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x14};

const REG_ADDRESS I2C_MST_ODR_CONFIG_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x00};
const REG_ADDRESS I2C_MST_CTRL_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x01};
const REG_ADDRESS I2C_SLV_ADDR_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x03};
const REG_ADDRESS I2C_SLV_REG_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x04};
const REG_ADDRESS I2C_SLV_CTRL_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x05};
const REG_ADDRESS I2C_SLV_DO_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x06};
