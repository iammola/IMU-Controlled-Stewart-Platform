#include <stdbool.h>

#include "ICM-20948.h"
#include "SPI/SPI.h"

#include "tm4c123gh6pm.h"

#define READ(addr)        (uint16_t)(0x80FF | (addr << 8))
#define WRITE(addr, data) (uint16_t)(0x7FFF & ((addr << 8) | data))

static DELAY_FUNC IMU_Delay;
static USER_BANK  LastUserBank = 0xFF;

uint8_t MAG_WHO_AM_I = 0x01;
uint8_t MAG_ST1 = 0x10;
uint8_t MAG_ST2 = 0x18;
uint8_t MAG_CNTL2 = 0x31;
uint8_t MAG_CNTL3 = 0x32;

REG_ADDRESS WHO_AM_I_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x00};
REG_ADDRESS GYRO_CONFIG_1_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x01};
REG_ADDRESS USER_CTRL_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x03};
REG_ADDRESS PWR_MGMT_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x06};
REG_ADDRESS PWR_MGMT_2_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x07};
REG_ADDRESS ACCEL_CONFIG_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x14};
REG_ADDRESS USER_BANK_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x7F};
REG_ADDRESS LP_CONFIG_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x05};

REG_ADDRESS GYRO_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x33};
REG_ADDRESS GYRO_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x34};
REG_ADDRESS GYRO_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x35};
REG_ADDRESS GYRO_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x36};
REG_ADDRESS ACCEL_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2D};
REG_ADDRESS ACCEL_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2E};
REG_ADDRESS ACCEL_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2F};
REG_ADDRESS ACCEL_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x30};

REG_ADDRESS MAG_STATUS_1_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x3B}; // EXT_SLV_DATA_0
REG_ADDRESS MAG_XOUT_L_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x3C};   // EXT_SLV_DATA_1
REG_ADDRESS MAG_XOUT_H_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x3D};   // EXT_SLV_DATA_2
REG_ADDRESS MAG_YOUT_L_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x3E};   // EXT_SLV_DATA_3
REG_ADDRESS MAG_YOUT_H_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x3F};   // EXT_SLV_DATA_4

REG_ADDRESS I2C_MST_CTRL_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x01};
REG_ADDRESS I2C_MST_ODR_CONFIG_ADDR = {};

REG_ADDRESS I2C_SLV_ADDR_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x03};
REG_ADDRESS I2C_SLV_REG_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x04};
REG_ADDRESS I2C_SLV_CTRL_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x05};
REG_ADDRESS I2C_SLV_DO_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x06};

static void IMU_Config(void) {
  uint8_t whoAmI = 0;

  IMU_Write(PWR_MGMT_1_ADDR, DEVICE_RESET); // Reset the device
  IMU_Delay(101, -3);                       // Wait atleast 100ms after device reset

  do {
    IMU_Read(WHO_AM_I_ADDR, &whoAmI); // Read IMU Identifier
  } while (whoAmI != 0xEA);

  IMU_Write(PWR_MGMT_1_ADDR, CLKSEL_AUTO | TEMP_ENABLE); // Disables Sleep, Low-Power Mode and Temp Sensor. Auto selects clk
  IMU_Delay(40, -3);                                     // Wait atleast 35ms after waking from sleep

  IMU_Write(PWR_MGMT_2_ADDR, ~(ACCEL_DISABLE | GYRO_DISABLE)); // Enable the Accelerometer and Gyroscope
  IMU_Delay(40, -3);                                           // Wait atleast 35ms after enabling accel and gyro

  IMU_Write(USER_CTRL_ADDR, I2C_MST_ENABLE | SPI_ENABLE); // Enable SPI, reset I2C master for magnetometer
  IMU_Delay(101, -3);                                     // Wait atleast 100ms after enabling device master

  IMU_Write(GYRO_CONFIG_1_ADDR, GYRO_FS_SEL_1000); // Configure gyro scale to 1000dps
  IMU_Write(ACCEL_CONFIG_ADDR, ACCEL_FS_SEL_8G);   // Configure accelerometer scale to 8G

  IMU_Write(I2C_MST_CTRL_ADDR, I2C_MST_CLK_400K);      // Use Mast CLK = 345.60kHz, Duty Cycle = 46.67%
  IMU_Write(LP_CONFIG_ADDR, I2C_MST_ODR);              // Use I2C_MST_ODR_CONFIG_ADDR for mag sampling rate
  IMU_Write(I2C_MST_ODR_CONFIG_ADDR, I2C_MST_ODR_137); // Use 137Hz rate on magnetometer

  IMU_Mag_ReadWhoAMI();

  IMU_Mag_Write(MAG_CNTL3, MAG_RESET);
  IMU_Delay(30, -6);                         // Wait at least 20us
  IMU_Mag_Write(MAG_CNTL2, MAG_CONT_MODE_4); // Use 100 Hz sample rate
  IMU_Delay(30, -6);                         // Wait at least 20us

  IMU_Mag_StartDataRead(); // set mag addresses to read
  IMU_Delay(30, -6);       // Wait at least 20us
}

static void IMU_ChangeUserBank(REG_ADDRESS REGISTER) {
  uint16_t byte = WRITE(USER_BANK_ADDR.ADDRESS, REGISTER.USER_BANK);

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

static void IMU_Mag_ReadWhoAMI(void) {
  uint8_t whoAmI;

  IMU_Write(I2C_SLV_ADDR_ADDR, 0x80 | MAG_I2C_ADDRESS); // Read op and set Mag I2C address
  IMU_Write(I2C_SLV_REG_ADDR, MAG_WHO_AM_I);            // Set Magnetometer to read from Who Am I register
  IMU_Write(I2C_SLV_CTRL_ADDR, 0x80 | 0x01);            // Read 1 byte data

  do {
    IMU_Read(MAG_STATUS_1_ADDR, &whoAmI);
  } while (whoAmI != 0x09);
}

static void IMU_Mag_StartDataRead(void) {
  IMU_Write(I2C_SLV_ADDR_ADDR, 0x80 | MAG_I2C_ADDRESS);         // Read op and set Mag I2C address
  IMU_Write(I2C_SLV_REG_ADDR, MAG_ST1);                         // Set Magnetometer Address to start read from ST1 register
  IMU_Write(I2C_SLV_CTRL_ADDR, 0x80 | (MAG_ST2 - MAG_ST1 + 1)); // Read data from HXL to status register
}

static void IMU_Mag_Write(uint8_t MAG_ADDRESS, uint8_t data) {
  IMU_Write(I2C_SLV_ADDR_ADDR, 0x7F & MAG_I2C_ADDRESS); // Write op and set Mag I2C address
  IMU_Write(I2C_SLV_REG_ADDR, MAG_ADDRESS);             // Set Magnetometer Address to read
  IMU_Write(I2C_SLV_DO_ADDR, data);                     // Set data to write
  IMU_Write(I2C_SLV_CTRL_ADDR, 0x80 | 0x01);            // Enable 1-byte data write
}

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, DELAY_FUNC delay) {
  if (delay == 0) {
    while (1)
      ;
  }

  IMU_Delay = delay;

  SPI3_Init(SYS_CLK, SSI_CLK, SSI_CR0_FRF_MOTO | SSI_CR0_SPO | SSI_CR0_SPH, SSI_CR0_DSS_16); // Initialize the SPI pins
  IMU_Config();                                                                              // Configure the IMU configuration settings

  // Configure the Interrupt pin
  // IMU_Interrupt_Init();
}

void IMU_GetAccelReadings(Coords *dest) {
  uint8_t accelXH = 0;
  uint8_t accelXL = 0;
  uint8_t accelYH = 0;
  uint8_t accelYL = 0;

  IMU_Read(ACCEL_XOUT_H_ADDR, &accelXH);
  IMU_Read(ACCEL_XOUT_L_ADDR, &accelXL);
  IMU_Read(ACCEL_YOUT_H_ADDR, &accelYH);
  IMU_Read(ACCEL_YOUT_L_ADDR, &accelYL);

  dest->x = ((accelXH << 8) | accelXL) / ACCEL_FS_SEL_8G_SENSITIVITY;
  dest->y = ((accelYH << 8) | accelYL) / ACCEL_FS_SEL_8G_SENSITIVITY;
}

void IMU_GetGyroReadings(Coords *dest) {
  uint8_t gyroXH = 0;
  uint8_t gyroXL = 0;
  uint8_t gyroYH = 0;
  uint8_t gyroYL = 0;

  IMU_Read(GYRO_XOUT_H_ADDR, &gyroXH);
  IMU_Read(GYRO_XOUT_L_ADDR, &gyroXL);
  IMU_Read(GYRO_YOUT_H_ADDR, &gyroYH);
  IMU_Read(GYRO_YOUT_L_ADDR, &gyroYL);

  dest->x = ((gyroXH << 8) | gyroXL) / GYRO_FS_SEL_1000_SENSITIVITY;
  dest->y = ((gyroYH << 8) | gyroYL) / GYRO_FS_SEL_1000_SENSITIVITY;
}

void IMU_GetMagReadings(Coords *dest) {
  uint8_t status = 0;

  uint8_t magXH = 0;
  uint8_t magXL = 0;
  uint8_t magYH = 0;
  uint8_t magYL = 0;

  IMU_Read(MAG_STATUS_1_ADDR, &status);
  if (!(status & MAG_DATA_RDY)) // Check if there's new data
    return;

  IMU_Read(MAG_XOUT_H_ADDR, &magXH);
  IMU_Read(MAG_XOUT_L_ADDR, &magXL);
  IMU_Read(MAG_YOUT_H_ADDR, &magYH);
  IMU_Read(MAG_YOUT_L_ADDR, &magYL);

  dest->x = ((magXH << 8) | magXL) / 6.66;
  dest->y = ((magYH << 8) | magYL) / 6.66;
}
