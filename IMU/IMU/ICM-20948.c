#include <stdbool.h>
#include <stdio.h>

#include "CLI/CLI.h"
#include "ICM-20948.h"
#include "SPI/SPI.h"

#include "Fusion/Fusion.h" // Madgwick Sensor Fusion from Seb Madgwick
#include "SysTick.h"
#include "tm4c123gh6pm.h"

#define INT_BIT      (unsigned)(1 << 6) // (PD6) Interrupt Pin
#define INT_PCTL_M   (unsigned)GPIO_PCTL_PD6_M
#define INT_PRIORITY 1

#define READ(addr)        (uint16_t)(0x80FF | (addr << 8))
#define WRITE(addr, data) (uint16_t)(0x7FFF & ((addr << 8) | data))

static uint32_t  SYS_CLOCK;
static USER_BANK LastUserBank = 0xFF;

#define CLI_TXT_BUF 500

void GPIOD_Handler(void);

static void IMU_Config(void);
static void IMU_ChangeUserBank(REG_ADDRESS REGISTER);
static void IMU_Read(REG_ADDRESS REGISTER, uint8_t *dest);
static void IMU_Write(REG_ADDRESS REGISTER, uint8_t data);
static void IMU_Delay(uint32_t inSeconds, int32_t powerOf10);

static void IMU_Mag_Init(void);
static void IMU_Mag_Write(uint8_t MAG_ADDRESS, uint8_t data);
static void IMU_Mag_Read(uint8_t MAG_ADDRESS, uint8_t *dest, uint8_t length);

static void IMU_GetMagReadings(FusionVector *dest);
static void IMU_GetGyroReadings(FusionVector *dest);
static void IMU_GetAccelReadings(FusionVector *dest);

#define MAG_WHO_AM_I 0x01
#define MAG_HXL      0x11
#define MAG_ST2      0x18
#define MAG_CNTL2    0x31
#define MAG_CNTL3    0x32

static REG_ADDRESS WHO_AM_I_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x00};
static REG_ADDRESS USER_CTRL_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x03};
static REG_ADDRESS LP_CONFIG_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x05};
static REG_ADDRESS PWR_MGMT_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x06};
static REG_ADDRESS PWR_MGMT_2_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x07};
static REG_ADDRESS INT_PIN_CFG_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x0F};
static REG_ADDRESS INT_ENABLE_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x11};
static REG_ADDRESS INT_STATUS_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x1A};

static REG_ADDRESS ACCEL_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2D};
static REG_ADDRESS ACCEL_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2E};
static REG_ADDRESS ACCEL_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2F};
static REG_ADDRESS ACCEL_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x30};
static REG_ADDRESS GYRO_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x33};
static REG_ADDRESS GYRO_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x34};
static REG_ADDRESS GYRO_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x35};
static REG_ADDRESS GYRO_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x36};
static REG_ADDRESS EXT_SLV_DATA_0 = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x3B};  // EXT_SLV_DATA_0
static REG_ADDRESS EXT_SLV_DATA_23 = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x52}; // EXT_SLV_DATA_23

static REG_ADDRESS USER_BANK_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x7F};

static REG_ADDRESS GYRO_SMPLRT_DIV_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x00};
static REG_ADDRESS GYRO_CONFIG_1_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x01};
static REG_ADDRESS ODR_ALIGN_EN_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x09};
static REG_ADDRESS ACCEL_SMPLRT_DIV_1_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x10};
static REG_ADDRESS ACCEL_SMPLRT_DIV_2_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x11};
static REG_ADDRESS ACCEL_CONFIG_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x14};

static REG_ADDRESS I2C_MST_ODR_CONFIG_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x00};
static REG_ADDRESS I2C_MST_CTRL_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x01};
static REG_ADDRESS I2C_SLV_ADDR_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x03};
static REG_ADDRESS I2C_SLV_REG_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x04};
static REG_ADDRESS I2C_SLV_CTRL_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x05};
static REG_ADDRESS I2C_SLV_DO_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x06};

// Initialise algorithms
#define SAMPLE_RATE 1.1e3 // Gyro Sample Rate of 1.1kHz

static uint32_t timestamp = 0;
static uint32_t lastTimestamp = 0;

static FusionAhrs   ahrs;
static FusionOffset offset;
FusionEuler         euler;

bool HasNewIMUAngles = false;

static FusionVector gyroscope = {
    {0.0f, 0.0f, 0.0f}
};
static FusionVector accelerometer = {
    {0.0f, 0.0f, 1.0f}
};
static FusionVector magnetometer = {
    {1.0f, 0.0f, 0.0f}
};

void GPIOD_Handler(void) {
  uint8_t intStatus = 0;
  float   deltaTime = 0;

  // Ensure the interrupt is on the INT pin and is a DMP interrupt
  if (GPIO_PORTD_MIS_R & INT_BIT) {
    GPIO_PORTD_ICR_R |= INT_BIT; // Clear Interrupt
    IMU_Read(INT_STATUS_1_ADDR, &intStatus);

    if (!intStatus)
      return;

    timestamp = ST_CURRENT_R;

    // Read sensor data
    IMU_GetGyroReadings(&gyroscope);
    IMU_GetAccelReadings(&accelerometer);
    IMU_GetMagReadings(&magnetometer);

    // Apply calibration
    /*
      // Variables holding calibration data. All z-axis data set to 0. Gyro and Accel Offset could be applied
      // at the ICM level, but might as well do it with the Fusion algo.
      const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
      const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
      const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
      const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
      const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
      const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
      const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
      const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

      gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
      accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
      magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
    */

    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    deltaTime = (timestamp - lastTimestamp) / SYS_CLOCK;
    lastTimestamp = timestamp;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

    // Calculate euler angles
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    HasNewIMUAngles = true;
  }
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

static void IMU_Config(void) {
  uint8_t whoAmI = 0;
  uint8_t MAG_whoAmI = 0;
  uint8_t userCtrl = 0;

  IMU_Write(PWR_MGMT_1_ADDR, DEVICE_RESET | SLEEP_ENABLE); // Reset the device
  IMU_Delay(101, -3);                                      // Wait atleast 100ms after device reset

  IMU_Write(PWR_MGMT_1_ADDR, CLKSEL_AUTO | TEMP_ENABLE); // Disables Sleep, Low-Power Mode and Temp Sensor. Auto selects clk
  IMU_Delay(40, -3);                                     // Wait atleast 35ms after waking from sleep

  do {
    IMU_Read(WHO_AM_I_ADDR, &whoAmI); // Read IMU Identifier
  } while (whoAmI != 0xEA);

  IMU_Write(PWR_MGMT_2_ADDR, (uint8_t) ~(ACCEL_DISABLE | GYRO_DISABLE)); // Enable the Accelerometer and Gyroscope
  IMU_Delay(40, -3);                                                     // Wait atleast 35ms after enabling accel and gyro

  IMU_Write(ODR_ALIGN_EN_ADDR, ODR_ALIGN_ENABLE);              // Align output data rate
  IMU_Write(GYRO_SMPLRT_DIV_ADDR, 0x00);                       // Configure for max sample rate of 1.1kHz (1.1kHz / (1 + 0))
  IMU_Write(GYRO_CONFIG_1_ADDR, GYRO_FS_SEL_1000 | GYRO_DLPF); // Configure gyro scale to 1000dps and enable Low-pass filter
  IMU_Write(ACCEL_CONFIG_ADDR, ACCEL_FS_SEL_8G | ACCEL_DLPF);  // Configure accelerometer scale to 8G
  IMU_Write(ACCEL_SMPLRT_DIV_1_ADDR, 0x00);                    // Configure for max sample rate of 1.1kHz (1.1kHz / (1 + 0))
  IMU_Write(ACCEL_SMPLRT_DIV_2_ADDR, 0x00);                    // Configure for max sample rate of 1.1kHz (1.1kHz / (1 + 0))

  IMU_Read(USER_CTRL_ADDR, &userCtrl);
  IMU_Write(USER_CTRL_ADDR, (userCtrl | SPI_ENABLE) & ~(DMP_ENABLE | FIFO_ENABLE)); // Enable SPI

  IMU_Mag_Init(); // Enable I2C master for Magnetometer read

  do {
    IMU_Mag_Read(MAG_WHO_AM_I, &MAG_whoAmI, 1); // Confirm communication success
  } while (MAG_whoAmI != 0x09);

  // Specify the Interrupt pin is push-pull and is an active high pin (falling edge interrupt)
  // also forces the Interrupt to be cleared for the level to be reset and any Read operation to clear the INT_STATUS
   IMU_Write(INT_PIN_CFG_ADDR, (INT_LATCH_MANUAL_CLEAR | INT_READ_CLEAR) & ~(INT_ACTIVE_LOW | INT_OPEN_DRAIN));
   IMU_Write(INT_ENABLE_1_ADDR, RAW_DATA_INT_ENABLE); // Enable Raw Data interrupt
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

  IMU_Delay(50, -3); // Wait at least 50ms
  for (dataIdx = 0; dataIdx < length; dataIdx++) {
    EXT_REG.ADDRESS += dataIdx;
    IMU_Read(EXT_REG, dest); // Read data
  }

  IMU_Write(I2C_SLV_CTRL_ADDR, 0x00); // Stop transmission
}

static void IMU_Mag_Write(uint8_t MAG_ADDRESS, uint8_t data) {
  IMU_Write(I2C_SLV_ADDR_ADDR, 0x7F & MAG_I2C_ADDRESS); // Write op and set Mag I2C address
  IMU_Write(I2C_SLV_REG_ADDR, MAG_ADDRESS);             // Set Magnetometer Address to read
  IMU_Write(I2C_SLV_DO_ADDR, data);                     // Set data to write
  IMU_Write(I2C_SLV_CTRL_ADDR, 0x80 | 0x01);            // Enable 1-byte data write
}

static void IMU_Delay(uint32_t inSeconds, int32_t powerOf10) {
  uint32_t CLOCK = SYS_CLOCK;

  while (powerOf10 < 0) {
    CLOCK /= 10;
    ++powerOf10;
  }

  SysTick_Wait(inSeconds * CLOCK);
}

static void IMU_MadgwickFusion_Init(void) {
  const FusionAhrsSettings settings = {
      .convention = FusionConventionEnu,
      .gain = 4.5f,
      .gyroscopeRange = 1000, // gyroscope range in dps
      .accelerationRejection = 5.0f,
      .magneticRejection = 5.0f,
      .recoveryTriggerPeriod = 5 * 1 / SAMPLE_RATE, /* 5 seconds */
  };

  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  FusionAhrsSetSettings(&ahrs, &settings);
}

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK) {
  SYS_CLOCK = SYS_CLK;

  SysTick_Init(); // Initialize SysTick

  SPI3_Init(SYS_CLOCK, SSI_CLK, SSI_CR0_FRF_MOTO | SSI_CR0_SPO | SSI_CR0_SPH, SSI_CR0_DSS_16); // Initialize the SPI pins
  IMU_Config();                                                                                // Configure the IMU configuration settings

  IMU_MadgwickFusion_Init();
  IMU_Interrupt_Init(); // Configure the Interrupt pin
}

void IMU_GetAccelReadings(FusionVector *dest) {
  uint8_t accelXH = 0;
  uint8_t accelXL = 0;
  uint8_t accelYH = 0;
  uint8_t accelYL = 0;

  IMU_Read(ACCEL_XOUT_H_ADDR, &accelXH);
  IMU_Read(ACCEL_XOUT_L_ADDR, &accelXL);
  IMU_Read(ACCEL_YOUT_H_ADDR, &accelYH);
  IMU_Read(ACCEL_YOUT_L_ADDR, &accelYL);

  dest->axis.x = (int16_t)((accelXH << 8) | accelXL) / ACCEL_FS_SEL_8G_SENSITIVITY;
  dest->axis.y = (int16_t)((accelYH << 8) | accelYL) / ACCEL_FS_SEL_8G_SENSITIVITY;
  dest->axis.z = 0; // Ignore z-axis
}

void IMU_GetGyroReadings(FusionVector *dest) {
  uint8_t gyroXH = 0;
  uint8_t gyroXL = 0;
  uint8_t gyroYH = 0;
  uint8_t gyroYL = 0;

  IMU_Read(GYRO_XOUT_H_ADDR, &gyroXH);
  IMU_Read(GYRO_XOUT_L_ADDR, &gyroXL);
  IMU_Read(GYRO_YOUT_H_ADDR, &gyroYH);
  IMU_Read(GYRO_YOUT_L_ADDR, &gyroYL);

  dest->axis.x = (int16_t)((gyroXH << 8) | gyroXL) / GYRO_FS_SEL_1000_SENSITIVITY;
  dest->axis.y = (int16_t)((gyroYH << 8) | gyroYL) / GYRO_FS_SEL_1000_SENSITIVITY;
  dest->axis.z = 0; // Ignore z-axis
}

void IMU_GetMagReadings(FusionVector *dest) {
  uint8_t ST2 = 0;
  uint8_t magCoords[4] = {0, 0, 0, 0};

  IMU_Mag_Read(MAG_HXL, magCoords, 4); // Get the X,Y bytes data
  IMU_Mag_Read(MAG_ST2, &ST2, 1);      // ST2 is required to be read to denote end

  dest->axis.x = (int16_t)((magCoords[1] << 8) | magCoords[0]) / MAG_4912_SENSITIVITY;
  dest->axis.y = (int16_t)((magCoords[3] << 8) | magCoords[2]) / MAG_4912_SENSITIVITY;
  dest->axis.z = 0; // Ignore z-axis
}
