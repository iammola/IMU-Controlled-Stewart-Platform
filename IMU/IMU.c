#include "tm4c123gh6pm.h"

#include "IMU.h"
#include "ICM-20948.h"

#define CLK 0 // (PD0) SSI3CLK
#define FSS 1 // (PD1) SSI3FSS (Chip Select)
#define RX 2  // (PD2) SSI3Rx
#define TX 3  // (PD3) SSI3Tx

#define SPI_PINS (unsigned)((1 << CLK) | (1 << FSS) | (1 << RX) | (1 << TX))
#define SPI_PCTL (unsigned)(GPIO_PCTL_PD0_SSI3CLK | GPIO_PCTL_PD1_SSI3FSS | GPIO_PCTL_PD2_SSI3RX | GPIO_PCTL_PD3_SSI3TX)
#define SPI_PCTL_MASK (unsigned)(GPIO_PCTL_PD0_M | GPIO_PCTL_PD1_M | GPIO_PCTL_PD2_M | GPIO_PCTL_PD3_M)

#define INT_BIT (unsigned)0x40 // (PD6) Interrupt Pin
#define INT_PCTL_M (unsigned)GPIO_PCTL_PD6_M
#define INT_PRIORITY 2

#define READ(addr) (unsigned)(0x8000 | (addr << 8))
#define WRITE(addr, data) (unsigned)(0x7FFF & ((addr << 8) | data))

// Defaults to USER_BANK_0
static USER_BANK LastUserBank = USER_BANK_0;

void GPIOD_Handler(void)
{
  uint8_t data;
  uint8_t intStatus = IMU_Read(INT_STATUS);

  // Ensure the interrupt is on the INT pin and is a DMP interrupt
  if ((GPIO_PORTD_MIS_R & INT_BIT) && (intStatus & DMP_INT))
  {
    // Get DMP data from FIFO register
    data = IMU_Read(FIFO_R_W);

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

  // Trigger interrupt on Falling edge
  GPIO_PORTD_IEV_R &= ~INT_BIT;

  // Clear the INT pin's interrupt
  GPIO_PORTD_ICR_R |= INT_BIT;

  // Allow the INT pin interrupt to be detected
  GPIO_PORTD_IM_R |= INT_BIT;

  // Enable Port D's Interrupt Handler
  NVIC_EN0_R |= NVIC_EN0_INT3;

  // Configure Port D's priority
  NVIC_PRI0_R = (NVIC_PRI0_R & ~NVIC_PRI0_INT3_M) | (INT_PRIORITY << NVIC_PRI0_INT3_S);
}

static void IMU_Config(void)
{
  // Wake the device from sleep, disable the Temp sensor, Turn off low power mode
  // and auto-selected clock source
  IMU_Write(PWR_MGMT_1_ADDR, CLKSEL_AUTO & ~(SLEEP_ENABLE | TEMP_ENABLE | LP_ENABLE));

  // Enable the Accelerometer and Gyroscope
  IMU_Write(PWR_MGMT_2_ADDR, ACCEL_ENABLE | GYRO_ENABLE);

  // Enable DMP and FIFO, and disable I2C for SPI only
  IMU_Write(USER_CTRL_ADDR, DMP_ENABLE | FIFO_ENABLE | SPI_ENABLE);

  // Configure gyro scale to 2000dps
  IMU_Write(GYRO_CONFIG_1, GYRO_FS_SEL_2000);

  // Configure accelerometer scale to 16G
  IMU_Write(ACCEL_CONFIG, ACCEL_FS_SEL_16G);

  // Specify the Interrupt pin is not open-drain (push-pull) and is an active high pin (falling edge interrupt)
  // also forces the Interrupt to be cleared for the level to be reset and any Read operation to clear the INT_STATUS
  IMU_Write(INT_PIN_CFG, (INT_LATCH_MANUAL_CLEAR | INT_READ_CLEAR) & ~(INT_ACTIVE_LOW | INT_OPEN_DRAIN));

  // Enable DMP interrupt
  IMU_Write(INT_ENABLE, DMP_INT_ENABLE);
}

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
  GPIO_PORTD_AFSEL_R |= SPI_PINS;

  // Enable SSI module 3 peripheral functions
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & ~SPI_PCTL_MASK) | SPI_PCTL;

  // Enable Digital Mode on pins
  GPIO_PORTD_DEN_R |= SPI_PINS;

  // Disable Analog Mode on pins
  GPIO_PORTD_AMSEL_R &= ~SPI_PINS;

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
  // Initialize the SPI pins
  SPI_Init(SYS_CLK, SSI_CLK);

  // Configure the IMU configuration settings
  IMU_Config();

  // Configure the Interrupt pin
  IMU_Interrupt_Init();
}

uint8_t IMU_Read(REG_ADDRESS REGISTER)
{
  if (LastUserBank != REGISTER.USER_BANK)
  {
    LastUserBank = REGISTER.USER_BANK;
    // Write to device the USER_BANK to read from
    SSI3_DR_R = WRITE(USER_BANK_ADDR.ADDRESS, REGISTER.USER_BANK);
  }

  // Put a 1 on the R/W bit to specify read
  SSI3_DR_R = READ(REGISTER.ADDRESS);

  // Wait for Receive FIFO to not be empty
  while ((SSI3_SR_R & SSI_SR_RNE) == 0x00)
    ;

  // Return data from register
  return SSI3_DR_R;
}

void IMU_Write(REG_ADDRESS REGISTER, uint8_t data)
{
  if (LastUserBank != REGISTER.USER_BANK)
  {
    LastUserBank = REGISTER.USER_BANK;
    // Write to device the USER_BANK to read from
    SSI3_DR_R = WRITE(USER_BANK_ADDR.ADDRESS, REGISTER.USER_BANK);
  }

  // Put a 1 on the R/W bit to specify read
  SSI3_DR_R = WRITE(REGISTER.ADDRESS, data);
}
