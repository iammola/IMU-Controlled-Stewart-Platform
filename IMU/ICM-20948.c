#include <stdbool.h>
#include "tm4c123gh6pm.h"

#include "SPI/SPI.h"
#include "ICM-20948.h"

#define INT_BIT (unsigned)0x40 // (PD6) Interrupt Pin
#define INT_PCTL_M (unsigned)GPIO_PCTL_PD6_M
#define INT_PRIORITY 2

#define READ(addr) (uint16_t)(0x80FF | (addr << 8))
#define WRITE(addr, data) (uint16_t)(0x7FFF & ((addr << 8) | data))

static DELAY_FUNC IMU_Delay;
// Defaults to USER_BANK_0
static USER_BANK LastUserBank = USER_BANK_0;

void GPIOD_Handler(void)
{
  uint8_t dmp = 0;
  uint8_t intStatus = 0;

  // Ensure the interrupt is on the INT pin and is a DMP interrupt
  if (GPIO_PORTD_MIS_R & INT_BIT) {
    IMU_Read(INT_STATUS_ADDR, &intStatus);

    if (intStatus & DMP_INT) {
      // Get DMP data from FIFO register
      IMU_Read(FIFO_R_W_ADDR, &dmp);
    }

    // Clear Interrupt
    GPIO_PORTD_ICR_R |= INT_BIT;
  }
}

static void IMU_Interrupt_Init(void)
{
  // Enable Port D clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;

  // Disable Alternate Functions on CS pin
  GPIO_PORTD_AFSEL_R &= ~INT_BIT;

  // Disable Peripheral functions on PD6
  GPIO_PORTD_PCTL_R &= ~INT_PCTL_M;

  // Configure INT bit as input
  GPIO_PORTD_DIR_R &= ~INT_BIT;

  // Enable Digital Mode on pins
  GPIO_PORTD_DEN_R |= INT_BIT;

  // Disable Analog Mode on pins
  GPIO_PORTD_AMSEL_R &= ~INT_BIT;

  // Disable the INT pin interrupt
  GPIO_PORTD_IM_R &= ~INT_BIT;

  // Configure for Edge-Detect interrupts
  GPIO_PORTD_IS_R &= ~INT_BIT;

  // Only listen on one edge event
  GPIO_PORTD_IBE_R &= ~INT_BIT;

  // Trigger interrupt on rising edge
  GPIO_PORTD_IEV_R |= INT_BIT;

  // Enable Port D's Interrupt Handler
  NVIC_EN0_R |= NVIC_EN0_INT3;

  // Configure Port D's priority
  NVIC_PRI0_R = (NVIC_PRI0_R & ~NVIC_PRI0_INT3_M) | (INT_PRIORITY << NVIC_PRI0_INT3_S);

  // Clear the INT pin's interrupt
  GPIO_PORTD_ICR_R |= INT_BIT;

  // Allow the INT pin interrupt to be detected
  GPIO_PORTD_IM_R |= INT_BIT;
}

static void IMU_Config(void)
{
  // Reset the device
  IMU_Write(PWR_MGMT_1_ADDR, DEVICE_RESET);

  // Wait atleast 100ms after device reset
  IMU_Delay(110);

  // Wake the device from sleep, disable the Temp sensor, Turn off low power mode
  // and auto-selected clock source
  IMU_Write(PWR_MGMT_1_ADDR, CLKSEL_AUTO & ~(SLEEP_ENABLE | TEMP_ENABLE | LP_ENABLE));

  // Wait atleast 20us after device reset
  IMU_Delay(10);

  // Enable the Accelerometer and Gyroscope
  IMU_Write(PWR_MGMT_2_ADDR, 0x3F & ~(ACCEL_DISABLE | GYRO_DISABLE));

  // Wait atleast 20us after device reset
  IMU_Delay(10);

  // Enable DMP and FIFO, and disable I2C for SPI only
  IMU_Write(USER_CTRL_ADDR, DMP_ENABLE | FIFO_ENABLE | SPI_ENABLE);

  // Configure gyro scale to 2000dps
  IMU_Write(GYRO_CONFIG_1_ADDR, GYRO_FS_SEL_2000);

  // Configure accelerometer scale to 16G
  IMU_Write(ACCEL_CONFIG_ADDR, ACCEL_FS_SEL_16G);

  // Specify the Interrupt pin is not open-drain (push-pull) and is an active high pin (falling edge interrupt)
  // also forces the Interrupt to be cleared for the level to be reset and any Read operation to clear the INT_STATUS
  // IMU_Write(INT_PIN_CFG_ADDR, (INT_LATCH_MANUAL_CLEAR | INT_READ_CLEAR) & ~(INT_ACTIVE_LOW | INT_OPEN_DRAIN));

  // Enable DMP interrupt
  // IMU_Write(INT_ENABLE_ADDR, DMP_INT_ENABLE);
}

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, DELAY_FUNC delay)
{
  if (delay == 0) {
    while (1)
      ;
  }

  IMU_Delay = delay;

  // Initialize the SPI pins
  SPI3_Init(SYS_CLK, SSI_CLK, SSI_CR0_FRF_MOTO | SSI_CR0_SPO | SSI_CR0_SPH, SSI_CR0_DSS_16);

  // Configure the IMU configuration settings
  IMU_Config();

  // Configure the Interrupt pin
  // IMU_Interrupt_Init();
}

static void IMU_ChangeUserBank(REG_ADDRESS REGISTER)
{
  uint16_t byte = WRITE(USER_BANK_ADDR.ADDRESS, REGISTER.USER_BANK);

  SPI3_StartTransmission();
  SPI3_Write(&byte, 1);
  SPI3_EndTransmission();

  LastUserBank = REGISTER.USER_BANK;
}

void IMU_Read(REG_ADDRESS REGISTER, uint8_t *dest)
{
  uint16_t response;

  // Change User Bank if Needed
  if (LastUserBank != REGISTER.USER_BANK)
    IMU_ChangeUserBank(REGISTER);

  SPI3_StartTransmission();
  SPI3_Read(READ(REGISTER.ADDRESS), &response, 1);
  SPI3_EndTransmission();

  *dest = response & 0xFF;
}

void IMU_Write(REG_ADDRESS REGISTER, uint8_t data)
{
  uint16_t byte = WRITE(REGISTER.ADDRESS, data);

  // Change User Bank if Needed
  if (LastUserBank != REGISTER.USER_BANK)
    IMU_ChangeUserBank(REGISTER);

  SPI3_StartTransmission();
  SPI3_Write(&byte, 1);
  SPI3_EndTransmission();
}

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
REG_ADDRESS GYRO_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x33};
REG_ADDRESS GYRO_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x34};
REG_ADDRESS GYRO_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x35};
REG_ADDRESS GYRO_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x36};
REG_ADDRESS ACCEL_XOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2D};
REG_ADDRESS ACCEL_XOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2E};
REG_ADDRESS ACCEL_YOUT_H_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x2F};
REG_ADDRESS ACCEL_YOUT_L_ADDR = {.USER_BANK = USER_BANK_0, .ADDRESS = 0x30};
