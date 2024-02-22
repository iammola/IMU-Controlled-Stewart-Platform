#include <stdbool.h>

#include "ICM-20948.h"
#include "SPI/SPI.h"

#include "tm4c123gh6pm.h"

#define INT_BIT      (unsigned)0x40 // (PD6) Interrupt Pin
#define INT_PCTL_M   (unsigned)GPIO_PCTL_PD6_M
#define INT_PRIORITY 2

#define READ(addr)        (uint16_t)(0x80FF | (addr << 8))
#define WRITE(addr, data) (uint16_t)(0x7FFF & ((addr << 8) | data))

static DELAY_FUNC IMU_Delay;
static USER_BANK  LastUserBank = 0xFF;

uint8_t MAG_ST2 = 0x18;
uint8_t MAG_HXL = 0x11;
uint8_t MAG_CNTL2 = 0x31;
uint8_t MAG_CNTL3 = 0x32;

REG_ADDRESS WHO_AM_I_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x00};
REG_ADDRESS GYRO_CONFIG_1_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x01};
REG_ADDRESS USER_CTRL_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x03};
REG_ADDRESS PWR_MGMT_1_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x06};
REG_ADDRESS PWR_MGMT_2_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x07};
REG_ADDRESS INT_PIN_CFG_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x0F};
REG_ADDRESS INT_ENABLE_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x10};
REG_ADDRESS ACCEL_CONFIG_ADDR = {.USER_BANK = USER_BANK_2, .ADDRESS = 0x14};
REG_ADDRESS INT_STATUS_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x19};
REG_ADDRESS FIFO_R_W_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x72};
REG_ADDRESS USER_BANK_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x7F};
REG_ADDRESS LP_CONFIG_ADDR = {};

REG_ADDRESS GYRO_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x33};
REG_ADDRESS GYRO_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x34};
REG_ADDRESS GYRO_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x35};
REG_ADDRESS GYRO_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x36};
REG_ADDRESS ACCEL_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2D};
REG_ADDRESS ACCEL_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2E};
REG_ADDRESS ACCEL_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2F};
REG_ADDRESS ACCEL_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x30};

REG_ADDRESS MAG_XOUT_L_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x3B}; // EXT_SLV_DATA_0
REG_ADDRESS MAG_XOUT_H_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x3C}; // EXT_SLV_DATA_1
REG_ADDRESS MAG_YOUT_L_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x3D}; // EXT_SLV_DATA_2
REG_ADDRESS MAG_YOUT_H_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x3E}; // EXT_SLV_DATA_3

REG_ADDRESS I2C_MST_CTRL_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x01};
REG_ADDRESS I2C_MST_ODR_CONFIG_ADDR = {};

REG_ADDRESS I2C_SLV_ADDR_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x03};
REG_ADDRESS I2C_SLV_REG_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x04};
REG_ADDRESS I2C_SLV_CTRL_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x05};
REG_ADDRESS I2C_SLV_DO_ADDR = {.USER_BANK = USER_BANK_3, .ADDRESS = 0x06};

void GPIOD_Handler(void) {
  uint8_t dmp = 0;
  uint8_t intStatus = 0;

  // Ensure the interrupt is on the INT pin and is a DMP interrupt
  if (GPIO_PORTD_MIS_R & INT_BIT) {
    IMU_Read(INT_STATUS_ADDR, &intStatus);

    if (intStatus & DMP_INT) {
      IMU_Read(FIFO_R_W_ADDR, &dmp); // Get DMP data from FIFO register
    }

    GPIO_PORTD_ICR_R |= INT_BIT; // Clear Interrupt
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
  uint8_t tmp = 0;

  IMU_Write(PWR_MGMT_1_ADDR, DEVICE_RESET); // Reset the device
  IMU_Delay(101, -3);                       // Wait atleast 100ms after device reset

  // Wake the device from sleep, disable the Temp sensor, Turn off low power mode
  // and auto-selected clock source
  IMU_Write(PWR_MGMT_1_ADDR, CLKSEL_AUTO & ~(SLEEP_ENABLE | TEMP_ENABLE | LP_ENABLE));
  IMU_Delay(21, -6); // Wait atleast 20us after waking from sleep

  IMU_Write(PWR_MGMT_2_ADDR, 0x3F & ~(ACCEL_DISABLE | GYRO_DISABLE)); // Enable the Accelerometer and Gyroscope
  IMU_Delay(21, -6);                                                  // Wait atleast 20us after enabling accel and gyro

  IMU_Read(USER_CTRL_ADDR, &tmp);
  tmp |= I2C_MST_RST | DMP_ENABLE | FIFO_ENABLE | SPI_ENABLE; // Enable DMP and FIFO, and SPI, reset I2C master for magnetometer
  IMU_Write(USER_CTRL_ADDR, tmp);
  IMU_Delay(101, -3); // Wait atleast 100ms after enabling device master

  tmp |= I2C_MST_ENABLE; // Enable I2C master
  IMU_Write(USER_CTRL_ADDR, tmp);

  IMU_Write(I2C_MST_CTRL_ADDR, I2C_MST_CLK_400K);
  IMU_Delay(10, -3); // Wait atleast 100ms after resetting I2C master

  IMU_Write(LP_CONFIG_ADDR, I2C_MST_ODR);              // Use I2C_MST_ODR_CONFIG_ADDR for mag sampling rate
  IMU_Write(I2C_MST_ODR_CONFIG_ADDR, I2C_MST_ODR_137); // Use 137Hz rate on magnetometer

  IMU_Write(GYRO_CONFIG_1_ADDR, GYRO_FS_SEL_2000); // Configure gyro scale to 2000dps
  IMU_Write(ACCEL_CONFIG_ADDR, ACCEL_FS_SEL_16G);  // Configure accelerometer scale to 16G

  IMU_Mag_Write(MAG_CNTL3, MAG_RESET);
  IMU_Delay(10, -3);                          // Wait 10ms
  IMU_Mag_Write(MAG_CNTL2, MAG_CONT_MODE_4); // Use 100 Hz sample rate
  IMU_Delay(10, -3);                          // Wait 10ms
  IMU_Mag_StartRead();                       // start reading mag data

  // Specify the Interrupt pin is push-pull and is an active high pin (falling edge interrupt)
  // also forces the Interrupt to be cleared for the level to be reset and any Read operation to clear the INT_STATUS
  // IMU_Write(INT_PIN_CFG_ADDR, (INT_LATCH_MANUAL_CLEAR | INT_READ_CLEAR) & ~(INT_ACTIVE_LOW | INT_OPEN_DRAIN));

  // IMU_Write(INT_ENABLE_ADDR, DMP_INT_ENABLE); // Enable DMP interrupt
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

static void IMU_Mag_StartRead(void) {
  IMU_Write(I2C_SLV_ADDR_ADDR, 0x80 | MAG_I2C_ADDRESS); // Read op and set Mag I2C address
  IMU_Write(I2C_SLV_REG_ADDR, MAG_HXL);                 // Set Magnetometer Address to start read from HXL register
  IMU_Write(I2C_SLV_CTRL_ADDR, 0x80 | (MAG_ST2 - MAG_HXL + 1)); // Read data from HXL to status register
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

  dest->x = ((accelXH << 8) | accelXL);
  dest->y = ((accelYH << 8) | accelYL);
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

  dest->x = ((gyroXH << 8) | gyroXL);
  dest->y = ((gyroYH << 8) | gyroYL);
}

void IMU_GetMagReadings(Coords *dest) {
  uint8_t magXH = 0;
  uint8_t magXL = 0;
  uint8_t magYH = 0;
  uint8_t magYL = 0;

  IMU_Read(MAG_XOUT_H_ADDR, &magXH);
  IMU_Read(MAG_XOUT_L_ADDR, &magXL);
  IMU_Read(MAG_YOUT_H_ADDR, &magYH);
  IMU_Read(MAG_YOUT_L_ADDR, &magYL);

  dest->x = ((magXH << 8) | magXL);
  dest->y = ((magYH << 8) | magYL);
}
