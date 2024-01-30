#include "tm4c123gh6pm.h"

#include "IMU.h"
#include "ICM-20948.h"

#define CLK 0 // (PD0) SSI3CLK
#define FSS 1 // (PD1) SSI3FSS
#define RX 2  // (PD2) SSI3Rx
#define TX 3  // (PD3) SSI3Tx

#define CS 4 // (PD4) Chip Select
#define CS_ADDR (*((volatile uint32_t *)(0x40007000 | (1 << (CS + 2)))))

#define PINS (unsigned)((1 << CLK) | (1 << FSS) | (1 << RX) | (1 << TX))
#define PCTL (unsigned)(GPIO_PCTL_PD0_SSI3CLK | GPIO_PCTL_PD1_SSI3FSS | GPIO_PCTL_PD2_SSI3RX | GPIO_PCTL_PD3_SSI3TX)
#define PCTL_MASK (unsigned)(GPIO_PCTL_PD0_M | GPIO_PCTL_PD1_M | GPIO_PCTL_PD2_M | GPIO_PCTL_PD3_M | GPIO_PCTL_PD4_M)

#define READ(data) (unsigned)(0x8000 | (data << 8))
#define WRITE(addr, data) (unsigned)(0x7FFF & ((addr << 8) | data))

// Defaults to USER_BANK_0
static USER_BANK LastUserBank = USER_BANK_0;

// Using Port D
static void SPI_Init(uint32_t SYS_CLK, uint32_t SSI_CLK)
{
  uint32_t ssiSCR = 0;
  uint32_t ssiCPSR = 0;
  uint32_t maxBitRate = SYS_CLK / SSI_CLK;

  // Enable Port D clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;

  // Enable SSI module 3 clock
  SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R3;

  // Enable Alternate Functions on all pins but the CS pin
  GPIO_PORTD_AFSEL_R = (GPIO_PORTD_AFSEL_R | PINS) & (unsigned)~(1 << CS);

  // Enable SSI module 3 peripheral functions
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & ~PCTL_MASK) | PCTL;

  // Enable Digital Mode on pins
  GPIO_PORTD_DEN_R |= PINS;

  // Disable Analog Mode on pins
  GPIO_PORTD_AMSEL_R &= ~PINS;

  // Enable Pull-Up on Clock pin so is driven high when idle
  GPIO_PORTD_PUR_R |= (1 << CLK);

  // Disable SSI
  SSI3_CR1_R &= (unsigned)~SSI_CR1_SSE;

  // Enable Master mode
  SSI3_CR0_R &= (unsigned)~SSI_CR1_MS;

  // Configure for system clock
  SSI3_CC_R = SSI_CC_CS_SYSPLL;

  // As a master, the system clock must be 2 times faster than the SSI_CLK and the
  // SSI_CLK cannot be more than 25MHz
  if ((SYS_CLK < (SSI_CLK * 2)) || SSI_CLK > 25e6)
  {
    while (1)
      ;
  }

  do
  {
    // Increment Pre-scale Divisor by 2
    ssiCPSR += 2;

    // Calculate new clock rate
    ssiSCR = (maxBitRate / ssiCPSR) - 1;
  } while (ssiSCR > 255);

  // The SSI Clk frequency needs to be increased
  if (ssiCPSR > 254)
  {
    while (1)
      ;
  }

  // Set the calculated pre-scale divisor
  SSI3_CPSR_R = ssiCPSR;

  // Set the calculated clock rate, FreeScale Frame Format, Steady High Pulse when idle, and 16 bit data
  SSI3_CR0_R = (ssiSCR << SSI_CR0_SCR_S) | SSI_CR0_FRF_MOTO | SSI_CR0_SPO | SSI_CR0_DSS_16;

  // Enable SSI
  SSI3_CR1_R |= SSI_CR1_SSE;
}

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK)
{
  SPI_Init(SYS_CLK, SSI_CLK);

  // Wake the device from sleep, disable the Temp sensor, Turn off low power mode
  // and auto-selected clock source
  IMU_Write(PWR_MGMT_1_ADDR, CLKSEL_AUTO & ~(SLEEP_ENABLED | TEMP_ENABLED | LP_ENABLED));

  // Enable the Accelerometer and Gyroscope
  IMU_Write(PWR_MGMT_2_ADDR, ACCEL_ENABLED | GYRO_ENABLED);

  // Enable DMP and FIFO, and disable I2C for SPI only
  IMU_Write(USER_CTRL_ADDR, DMP_ENABLED | FIFO_ENABLED | SPI_ENABLED);

  // Configure gyro scale to 2000dps
  IMU_Write(GYRO_CONFIG_1, GYRO_FS_SEL_2000);

  // Configure accelerometer scale to 16G
  IMU_Write(ACCEL_CONFIG, ACCEL_FS_SEL_16G);
}

void IMU_Read(REG_ADDRESS REGISTER)
{
  // Drive Chip Select low for slave active
  CS_ADDR = 0;

  if (LastUserBank != REGISTER.USER_BANK)
  {
    LastUserBank = REGISTER.USER_BANK;
    // Write to device the USER_BANK to read from
    SSI3_DR_R = WRITE(USER_BANK_ADDR.ADDRESS, REGISTER.USER_BANK);
  }

  // Put a 1 on the R/W bit to specify read
  SSI3_DR_R = READ(REGISTER.ADDRESS);

  // Drive Chip Select high for slave idle
  CS_ADDR = 1 << CS;
}

void IMU_Write(REG_ADDRESS REGISTER, uint8_t data)
{
  // Drive Chip Select low for slave active
  CS_ADDR = 0;

  if (LastUserBank != REGISTER.USER_BANK)
  {
    LastUserBank = REGISTER.USER_BANK;
    // Write to device the USER_BANK to read from
    SSI3_DR_R = WRITE(USER_BANK_ADDR.ADDRESS, REGISTER.USER_BANK);
  }

  // Put a 1 on the R/W bit to specify read
  SSI3_DR_R = WRITE(REGISTER.ADDRESS, data);

  // Drive Chip Select high for slave idle
  CS_ADDR = 1 << CS;
}
