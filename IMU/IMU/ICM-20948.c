//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
//
// This file can be used by the Keil uVision configuration wizard to set
// the following system clock configuration values.  Or the value of the
// macros can be directly edited below if not using the uVision configuration
// wizard.
//
//--------------------- Clock Configuration ----------------------------------

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
#include "ICM-20948.h"
#include "SPI/SPI.h"

#include "Fusion/Fusion.h" // Madgwick Sensor Fusion from Seb Madgwick
#include "SysTick.h"
#include "tm4c123gh6pm.h"

#define READ(addr)        (uint16_t)(0x80FF | (addr << 8))
#define WRITE(addr, data) (uint16_t)(0x7FFF & ((addr << 8) | data))

static uint32_t  SYS_CLOCK;
static USER_BANK LastUserBank = 0xFF;

char text[CLI_TXT_BUF] = "";

void GPIOD_Handler(void);

static void IMU_ChangeUserBank(REG_ADDRESS REGISTER);
static void IMU_Read(REG_ADDRESS REGISTER, uint8_t *dest);
static void IMU_Write(REG_ADDRESS REGISTER, uint8_t data);
static void IMU_Delay(uint32_t inSeconds, int32_t powerOf10);

static void IMU_MagCalibration(void);
static void IMU_AccelGyroCalibration(void);

static void IMU_Mag_Init(void);
static void IMU_Mag_Write(uint8_t MAG_ADDRESS, uint8_t data);
static void IMU_Mag_Read(uint8_t MAG_ADDRESS, uint8_t *dest, uint8_t length);

static bool IMU_GetMagReadings(FusionVector *dest);
static void IMU_GetRawGyroReadings(FusionVector *dest);
static void IMU_GetRawAccelReadings(FusionVector *dest);

static const REG_ADDRESS WHO_AM_I_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x00};
static const REG_ADDRESS USER_CTRL_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x03};
static const REG_ADDRESS LP_CONFIG_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x05};
static const REG_ADDRESS PWR_MGMT_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x06};
static const REG_ADDRESS PWR_MGMT_2_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x07};
static const REG_ADDRESS INT_PIN_CFG_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x0F};
static const REG_ADDRESS INT_ENABLE_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x11};
static const REG_ADDRESS INT_STATUS_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x1A};

static const REG_ADDRESS ACCEL_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2D};
static const REG_ADDRESS ACCEL_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2E};
static const REG_ADDRESS ACCEL_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2F};
static const REG_ADDRESS ACCEL_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x30};
static const REG_ADDRESS ACCEL_ZOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x31};
static const REG_ADDRESS ACCEL_ZOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x32};
static const REG_ADDRESS GYRO_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x33};
static const REG_ADDRESS GYRO_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x34};
static const REG_ADDRESS GYRO_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x35};
static const REG_ADDRESS GYRO_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x36};
static const REG_ADDRESS GYRO_ZOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x37};
static const REG_ADDRESS GYRO_ZOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x38};
static const REG_ADDRESS EXT_SLV_DATA_0 = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x3B};
static const REG_ADDRESS EXT_SLV_DATA_23 = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x52};

static const REG_ADDRESS USER_BANK_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x7F};

static const REG_ADDRESS GYRO_SMPLRT_DIV_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x00};
static const REG_ADDRESS GYRO_CONFIG_1_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x01};
static const REG_ADDRESS ODR_ALIGN_EN_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x09};
static const REG_ADDRESS ACCEL_SMPLRT_DIV_1_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x10};
static const REG_ADDRESS ACCEL_SMPLRT_DIV_2_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x11};
static const REG_ADDRESS ACCEL_CONFIG_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x14};

static const REG_ADDRESS I2C_MST_ODR_CONFIG_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x00};
static const REG_ADDRESS I2C_MST_CTRL_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x01};
static const REG_ADDRESS I2C_SLV_ADDR_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x03};
static const REG_ADDRESS I2C_SLV_REG_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x04};
static const REG_ADDRESS I2C_SLV_CTRL_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x05};
static const REG_ADDRESS I2C_SLV_DO_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x06};

// Initialise algorithms
#define SAMPLE_RATE 100 // Gyro Sample Rate of 100 Hz

FusionAhrs   ahrs = {0};
FusionOffset offset = {0};
FusionEuler  euler = {0};

volatile bool HasNewIMUAngles = false;

volatile float           deltaTime = 0.0f;
static volatile uint32_t lastTimestamp = 0;

#define SENSITIVITY(scale) scale / (1 << 15)
static const float ACCEL_8G_SENSITIVITY = SENSITIVITY(8.0f);
static const float ACCEL_2G_SENSITIVITY = SENSITIVITY(2.0f);
static const float MAG_4912_SENSITIVITY = SENSITIVITY(4912.0f);
static const float GYRO_250_SENSITIVITY = SENSITIVITY(250.0f);
static const float GYRO_1000_SENSITIVITY = SENSITIVITY(1000.0f);
static const float GYRO_2000_SENSITIVITY = SENSITIVITY(2000.0f);

FusionVector rawGyroscope = {
    .axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}
};
FusionVector rawAccelerometer = {
    .axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}
};
FusionVector rawMagnetometer = {
    .axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}
};

void GPIOD_Handler(void) {
  uint8_t           intStatus = 0;
  volatile uint32_t timestamp = ST_CURRENT_R;

  GPIO_PORTD_ICR_R |= INT_BIT; // Clear Interrupt

  IMU_Read(INT_STATUS_1_ADDR, &intStatus);
  if (!intStatus)
    return;

  // Read sensor data
  IMU_GetRawGyroReadings(&rawGyroscope);
  IMU_GetRawAccelReadings(&rawAccelerometer);
  IMU_GetMagReadings(&rawMagnetometer);

  if (lastTimestamp == 0)
    deltaTime = 0.0f; // Start of process
  else if (lastTimestamp < timestamp)
    deltaTime = ST_RELOAD_R - (timestamp - lastTimestamp); // Restarted at max reload value, so have to find ticks in reverse
  else
    deltaTime = lastTimestamp - timestamp; // Ticks since last interrupt

  deltaTime /= (float)(SYS_CLOCK); // Calculate delta time (in seconds) to account for gyroscope sample clock error
  lastTimestamp = timestamp;       // Update timestamp tracker

  HasNewIMUAngles = true;
}

static void IMU_Interrupt_Init(void) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // Enable Port D clock

  GPIO_PORTD_AFSEL_R &= ~INT_BIT;   // Disable Alternate Functions on CS pin
  GPIO_PORTD_PCTL_R &= ~INT_PCTL_M; // Disable Peripheral functions on PD6
  GPIO_PORTD_DIR_R &= ~INT_BIT;     // Configure INT bit as input
  GPIO_PORTD_DEN_R |= INT_BIT;      // Enable Digital Mode on pins
  GPIO_PORTD_AMSEL_R &= ~INT_BIT;   // Disable Analog Mode on pins
  GPIO_PORTD_IM_R &= ~INT_BIT;      // Disable the INT pin interrupt
  GPIO_PORTD_IS_R &= ~INT_BIT;      // Configure for Edge-Detect interrupts
  GPIO_PORTD_IBE_R &= ~INT_BIT;     // Only listen on one edge event
  GPIO_PORTD_IEV_R |= INT_BIT;      // Trigger interrupt on rising edge

  NVIC_EN0_R |= NVIC_EN0_INT3;                                                          // Enable Port D's Interrupt Handler
  NVIC_PRI0_R = (NVIC_PRI0_R & ~NVIC_PRI0_INT3_M) | (INT_PRIORITY << NVIC_PRI0_INT3_S); // Configure Port D's priority
  GPIO_PORTD_ICR_R |= INT_BIT;                                                          // Clear the INT pin's interrupt
  GPIO_PORTD_IM_R |= INT_BIT;                                                           // Allow the INT pin interrupt to be detected
}

static void IMU_ChangeUserBank(REG_ADDRESS REGISTER) {
  uint16_t byte = WRITE(USER_BANK_ADDR.ADDRESS, REGISTER.USER_BANK);

  if (REGISTER.ADDRESS == USER_BANK_ADDR.ADDRESS)
    return;

  SPI3_StartTransmission();
  SPI3_Write(&byte, 1);
  SPI3_EndTransmission();

  LastUserBank = REGISTER.USER_BANK;
}

static void IMU_Read(REG_ADDRESS REGISTER, uint8_t *dest) {
  uint16_t response;

  // Change User Bank if Needed
  if (LastUserBank != REGISTER.USER_BANK)
    IMU_ChangeUserBank(REGISTER);

  SPI3_StartTransmission();
  SPI3_Read(READ(REGISTER.ADDRESS), &response, 1);
  SPI3_EndTransmission();

  *dest = response & 0xFF;
}

static void IMU_Write(REG_ADDRESS REGISTER, uint8_t data) {
  uint16_t byte = WRITE(REGISTER.ADDRESS, data);

  // Change User Bank if Needed
  if (LastUserBank != REGISTER.USER_BANK)
    IMU_ChangeUserBank(REGISTER);

  SPI3_StartTransmission();
  SPI3_Write(&byte, 1);
  SPI3_EndTransmission();
}

static void IMU_Mag_Init(void) {
  uint8_t tmp = 0;
  uint8_t userCtrl = 0x00;
  IMU_Read(USER_CTRL_ADDR, &userCtrl); // Get current user ctrl settings

  IMU_Write(USER_CTRL_ADDR, userCtrl | I2C_MST_RST); // Reset I2C master
  IMU_Delay(100, -3);                                // wait for 100ms after I2C master reset

  IMU_Write(USER_CTRL_ADDR, userCtrl | I2C_MST_ENABLE); // enable I2C master
  IMU_Write(I2C_MST_CTRL_ADDR, I2C_MST_CLK_400K);       // Use Mast CLK = 345.60kHz, Duty Cycle = 46.67%
  IMU_Write(LP_CONFIG_ADDR, I2C_MST_ODR);               // Use I2C_MST_ODR_CONFIG_ADDR for mag sampling rate
  IMU_Write(I2C_MST_ODR_CONFIG_ADDR, I2C_MST_ODR_137);  // Use 137Hz rate on magnetometer

  IMU_Mag_Write(MAG_CNTL3, MAG_RESET); // reset mag
  IMU_Delay(105, -6);                  // Wait at least 100us

  do {                                         // Poll until in Continuous Mode 4
    IMU_Mag_Write(MAG_CNTL2, MAG_CONT_MODE_4); // Use 100 Hz sample rate
    IMU_Delay(105, -6);                        // Wait at least 100us

    IMU_Mag_Read(MAG_CNTL2, &tmp, 1);
  } while (tmp != MAG_CONT_MODE_4);
}

static void IMU_Mag_Read(uint8_t MAG_ADDRESS, uint8_t *dest, uint8_t length) {
  uint8_t     dataIdx = 0;
  REG_ADDRESS EXT_REG = EXT_SLV_DATA_0;

  if (length > (EXT_SLV_DATA_23.ADDRESS - EXT_SLV_DATA_0.ADDRESS)) {
    return;
  }

  IMU_Write(I2C_SLV_ADDR_ADDR, 0x80 | MAG_I2C_ADDRESS); // Read op and set Mag I2C address
  IMU_Write(I2C_SLV_REG_ADDR, MAG_ADDRESS);             // Set Magnetometer to start read from desired register
  IMU_Write(I2C_SLV_CTRL_ADDR, 0x80 | length);          // Allow transmission for x bytes of data

  IMU_Delay(10, -6); // Wait 10 us

  for (dataIdx = 0; dataIdx < length; dataIdx++) {
    IMU_Read(EXT_REG, dest); // Read data
    EXT_REG.ADDRESS++;
    dest++;
  }
}

static void IMU_Mag_Write(uint8_t MAG_ADDRESS, uint8_t data) {
  IMU_Write(I2C_SLV_ADDR_ADDR, 0x7F & MAG_I2C_ADDRESS); // Write op and set Mag I2C address
  IMU_Write(I2C_SLV_REG_ADDR, MAG_ADDRESS);             // Set Magnetometer Address to read
  IMU_Write(I2C_SLV_DO_ADDR, data);                     // Set data to write
  IMU_Write(I2C_SLV_CTRL_ADDR, 0x80 | 0x01);            // Enable 1-byte data write
}

static void IMU_Delay(uint32_t inSeconds, int32_t powerOf10) {
  uint32_t CLOCK = (uint32_t)SYS_CLOCK;

  while (powerOf10 < 0) {
    CLOCK /= 10;
    ++powerOf10;
  }

  SysTick_Wait(inSeconds * CLOCK);
}

static void IMU_MadgwickFusion_Init(void) {
  const FusionAhrsSettings settings = {
      .convention = FusionConventionEnu,
      .gain = 1.5f,
      .gyroscopeRange = 2000, // gyroscope range in dps
      .accelerationRejection = 5.0f,
      .magneticRejection = 5.0f,
      .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
  };

  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  FusionAhrsSetSettings(&ahrs, &settings);
}

static void IMU_MagCalibration(void) {
// MotionCal uses 115200 Baud
#if (CALIBRATION_MODE == 5) || (CALIBRATION_MODE == 6)
  uint8_t      magStatus = 0;
  FusionVector magSample = {0};
  FusionMatrix sampleSoftIronMatrix = {
      .array = {{1.f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
  };
  FusionVector sampleHardIronOffset = {
      .axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}
  };

  uint8_t calibrationDataIdx = 0;
  uint8_t calibrationData[68] = {0};

  float magField = 0.0f;
  float offsets[10] = {0};

  while (1) {
    if (!IMU_GetMagReadings(&magSample))
      continue;
#if CALIBRATION_MODE == 6
    magSample = FusionCalibrationMagnetic(magSample, sampleSoftIronMatrix, sampleHardIronOffset);
#endif

    // Suggested to multiply by 10 for "Good Integer Value"
    snprintf(text, CLI_TXT_BUF, "Raw:0,0,0,0,0,0,%d,%d,%d\n\r", (int)(magSample.axis.x * 10), (int)(magSample.axis.y * 10),
             (int)(magSample.axis.z * 10));
    CLI_Write(text);

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

static void IMU_AccelGyroCalibration(void) {
#if (CALIBRATION_MODE > 0) && (CALIBRATION_MODE < 5)
  const int32_t SAMPLES_COUNT = 5e4;

  FusionVector sample = {
      .axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}
  };
  FusionVector totalSamples = {
      .axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}
  };
  FusionVector sampleOffset = {
      .axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}
  };
  FusionVector sampleSensitivity = {
      .axis = {.x = 1.0f, .y = 1.0f, .z = 1.0f}
  };
  FusionMatrix sampleMisalignment = {
      .array = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
  };
  uint32_t SAMPLES_LEFT = 0;

#if (CALIBRATION_MODE == 3) || (CALIBRATION_MODE == 1)
  // Reset offset if not in test mode, for new calibration values
  sampleOffset.axis.x = sampleOffset.axis.y = sampleOffset.axis.z = 0;
#endif

#if (CALIBRATION_MODE == 3) || (CALIBRATION_MODE == 4)
  IMU_Write(GYRO_CONFIG_1_ADDR, GYRO_DLPF_12HZ | GYRO_FS_SEL_250 | GYRO_DLPF_ENABLE); // Most sensitive scale 250dps, 200Hz BW
  sampleSensitivity.axis.x = sampleSensitivity.axis.y = sampleSensitivity.axis.z = GYRO_250_SENSITIVITY;
#elif (CALIBRATION_MODE == 1) || (CALIBRATION_MODE == 2)
  IMU_Write(ACCEL_CONFIG_ADDR, ACCEL_DLPF_246Hz | ACCEL_FS_SEL_2G | ACCEL_DLPF_ENABLE); // Most sensitive range 2G full scale, 246Hz BW
  sampleSensitivity.axis.x = sampleSensitivity.axis.y = sampleSensitivity.axis.z = ACCEL_2G_SENSITIVITY;
#endif

  for (SAMPLES_LEFT = SAMPLES_COUNT; SAMPLES_LEFT > 0; SAMPLES_LEFT--) {
#if (CALIBRATION_MODE == 3) || (CALIBRATION_MODE == 4)
    IMU_GetRawGyroReadings(&sample);
#elif (CALIBRATION_MODE == 1) || (CALIBRATION_MODE == 2)
    IMU_GetRawAccelReadings(&sample);
#endif
    sample = FusionCalibrationInertial(sample, sampleMisalignment, sampleSensitivity, sampleOffset);

    // This will be in the unit from the sensitivity
    totalSamples.axis.x += sample.axis.x;
    totalSamples.axis.y += sample.axis.y;
    totalSamples.axis.z += sample.axis.z;

    snprintf(text, CLI_TXT_BUF, "%0.4f %0.4f %0.4f\n", sample.axis.x, sample.axis.y, sample.axis.z);
    CLI_Write(text);
  }

  // Normalize back to LSBs
  totalSamples.axis.x /= (SAMPLES_COUNT * sampleSensitivity.axis.x);
  totalSamples.axis.y /= (SAMPLES_COUNT * sampleSensitivity.axis.y);
  totalSamples.axis.z /= (SAMPLES_COUNT * sampleSensitivity.axis.z);

  snprintf(text, CLI_TXT_BUF, "Measurement finished.\nX= %0.4f Y = %0.4f Z = %0.4f", totalSamples.axis.x, totalSamples.axis.y, totalSamples.axis.z);
  CLI_Write(text);

  while (1)
    ;
#endif
}

static void IMU_GetRawAccelReadings(FusionVector *dest) {
  uint8_t accelXH = 0;
  uint8_t accelXL = 0;
  uint8_t accelYH = 0;
  uint8_t accelYL = 0;
  uint8_t accelZH = 0;
  uint8_t accelZL = 0;

  IMU_Read(ACCEL_XOUT_H_ADDR, &accelXH);
  IMU_Read(ACCEL_XOUT_L_ADDR, &accelXL);
  IMU_Read(ACCEL_YOUT_H_ADDR, &accelYH);
  IMU_Read(ACCEL_YOUT_L_ADDR, &accelYL);
  IMU_Read(ACCEL_ZOUT_H_ADDR, &accelZH);
  IMU_Read(ACCEL_ZOUT_L_ADDR, &accelZL);

  dest->axis.x = (int16_t)((accelXH << 8) | accelXL);
  dest->axis.y = (int16_t)((accelYH << 8) | accelYL);
  dest->axis.z = (int16_t)((accelZH << 8) | accelZL);
}

static void IMU_GetRawGyroReadings(FusionVector *dest) {
  uint8_t gyroXH = 0;
  uint8_t gyroXL = 0;
  uint8_t gyroYH = 0;
  uint8_t gyroYL = 0;
  uint8_t gyroZH = 0;
  uint8_t gyroZL = 0;

  IMU_Read(GYRO_XOUT_H_ADDR, &gyroXH);
  IMU_Read(GYRO_XOUT_L_ADDR, &gyroXL);
  IMU_Read(GYRO_YOUT_H_ADDR, &gyroYH);
  IMU_Read(GYRO_YOUT_L_ADDR, &gyroYL);
  IMU_Read(GYRO_ZOUT_H_ADDR, &gyroZH);
  IMU_Read(GYRO_ZOUT_L_ADDR, &gyroZL);

  dest->axis.x = (int16_t)((gyroXH << 8) | gyroXL);
  dest->axis.y = (int16_t)((gyroYH << 8) | gyroYL);
  dest->axis.z = (int16_t)((gyroZH << 8) | gyroZL);
}

static bool IMU_GetMagReadings(FusionVector *dest) {
  uint8_t ST1 = 0;
  uint8_t magCoords[8] = {0};

  IMU_Mag_Read(MAG_ST1, &ST1, 1); // Get the status

  if (ST1 & MAG_DATA_RDY) {
    IMU_Delay(10, -6);
    IMU_Mag_Read(MAG_HXL, magCoords, 8); // Get the X,Y,Z bytes data and ST2 required for read end

    dest->axis.x = (int16_t)((magCoords[1] << 8) | magCoords[0]) * MAG_4912_SENSITIVITY;
    dest->axis.y = (int16_t)((magCoords[3] << 8) | magCoords[2]) * MAG_4912_SENSITIVITY * -1;
    dest->axis.z = (int16_t)((magCoords[5] << 8) | magCoords[4]) * MAG_4912_SENSITIVITY * -1;

    return true;
  } else if (ST1 & MAG_DATA_OVRRUN) {
    IMU_Mag_Read(MAG_ST2, &ST1, 1); // Read ST2
  }

  return false;

  // magCoords[7] = STATUS_2
}

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, FusionVector *gyroSensitivity, FusionVector *gyroOffset, FusionVector *accelSensitivity,
              FusionVector *accelOffset) {
  uint8_t whoAmI = 0;
  uint8_t MAG_whoAmI = 0;
  uint8_t userCtrl = 0;

  SYS_CLOCK = SYS_CLK;

  SysTick_Init(); // Initialize SysTick

  SPI3_Init(SYS_CLOCK, SSI_CLK, SSI_CR0_FRF_MOTO | SSI_CR0_SPO | SSI_CR0_SPH, SSI_CR0_DSS_16); // Initialize the SPI pins

  IMU_Write(PWR_MGMT_1_ADDR, DEVICE_RESET | SLEEP_ENABLE); // Reset the device
  IMU_Delay(101, -3);                                      // Wait atleast 100ms after device reset

  IMU_Write(PWR_MGMT_1_ADDR, CLKSEL_AUTO | TEMP_ENABLE); // Disables Sleep, Low-Power Mode and Temp Sensor. Auto selects clk
  IMU_Delay(40, -3);                                     // Wait atleast 35ms after waking from sleep

  IMU_Write(PWR_MGMT_2_ADDR, (uint8_t) ~(ACCEL_DISABLE | GYRO_DISABLE)); // Enable the Accelerometer and Gyroscope
  IMU_Delay(40, -3);                                                     // Wait atleast 35ms after enabling accel and gyro

  IMU_Write(GYRO_CONFIG_1_ADDR, GYRO_FS_SEL_2000 | GYRO_DLPF_ENABLE); // Configure gyro scale to 2000dps and enable Low-pass filter
  gyroSensitivity->axis.x = gyroSensitivity->axis.y = gyroSensitivity->axis.z = GYRO_2000_SENSITIVITY;
  // Change offset scale from min 250 dps to chosen gyro scale
  gyroOffset->axis.x /= 2000 / 250;
  gyroOffset->axis.y /= 2000 / 250;
  gyroOffset->axis.z /= 2000 / 250;

  IMU_Write(ACCEL_CONFIG_ADDR, ACCEL_FS_SEL_8G | ACCEL_DLPF_ENABLE); // Configure accelerometer scale to 8G
  accelSensitivity->axis.x = accelSensitivity->axis.y = accelSensitivity->axis.z = ACCEL_8G_SENSITIVITY;
  // Change offset scale from min 2G to chosen accel scale
  accelOffset->axis.x /= 8 / 2;
  accelOffset->axis.y /= 8 / 2;
  accelOffset->axis.z /= 8 / 2;

  IMU_Write(ODR_ALIGN_EN_ADDR, ODR_ALIGN_ENABLE); // Align output data rate
  IMU_Write(GYRO_SMPLRT_DIV_ADDR, 0x0A);          // Configure for sample rate of 100 Hz (1.1kHz / (1 + 10))
  IMU_Write(ACCEL_SMPLRT_DIV_1_ADDR, 0x00);       // Configure for max sample rate of 100 Hz (1.1kHz / (1 + 10))
  IMU_Write(ACCEL_SMPLRT_DIV_2_ADDR, 0x0A);       // Configure for max sample rate of 100 Hz (1.1kHz / (1 + 10))

  IMU_Read(USER_CTRL_ADDR, &userCtrl);
  IMU_Write(USER_CTRL_ADDR, (userCtrl | SPI_ENABLE) & ~(DMP_ENABLE | FIFO_ENABLE)); // Enable SPI

  IMU_Mag_Init(); // Enable I2C master for Magnetometer read

  do {
    IMU_Read(WHO_AM_I_ADDR, &whoAmI); // Read IMU Identifier
  } while (whoAmI != 0xEA);

  do {
    IMU_Mag_Read(MAG_WHO_AM_I, &MAG_whoAmI, 1); // Confirm communication success
  } while (MAG_whoAmI != 0x09);

  IMU_MagCalibration();       // Run Calibrations
  IMU_AccelGyroCalibration(); // Will be empty functions if undesired

  // Specify the Interrupt pin is push-pull and is an active high pin (falling edge interrupt)
  // also forces the Interrupt to be cleared for the level to be reset and any Read operation to clear the INT_STATUS
  IMU_Write(INT_PIN_CFG_ADDR, INT_READ_CLEAR & ~(INT_ACTIVE_LOW | INT_OPEN_DRAIN | INT_LATCH_MANUAL_CLEAR));
  IMU_Write(INT_ENABLE_1_ADDR, RAW_DATA_INT_ENABLE); // Enable Raw Data interrupt

  IMU_MadgwickFusion_Init();
  IMU_Interrupt_Init(); // Configure the Interrupt pin
}
