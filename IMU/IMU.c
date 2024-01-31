// TODO: Remove the CLK_BIT from the interrupt handler
//       Only added to attempt to watch the pins because the logic
//       analyzer wasn't working.
#include "tm4c123gh6pm.h"

#include "IMU.h"
#include "ICM-20948.h"

#define CLK_BIT 1 << 0 // (PD0) SSI3CLK
#define FSS_BIT 1 << 1 // (PD1) SSI3FSS (Chip Select)
#define RX_BIT 1 << 2  // (PD2) SSI3Rx
#define TX_BIT 1 << 3  // (PD3) SSI3Tx

#define SPI_PINS (unsigned)(CLK_BIT | FSS_BIT | RX_BIT | TX_BIT)
#define SPI_PCTL (unsigned)(GPIO_PCTL_PD0_SSI3CLK | GPIO_PCTL_PD1_SSI3FSS | GPIO_PCTL_PD2_SSI3RX | GPIO_PCTL_PD3_SSI3TX)
#define SPI_PCTL_MASK (unsigned)(GPIO_PCTL_PD0_M | GPIO_PCTL_PD1_M | GPIO_PCTL_PD2_M | GPIO_PCTL_PD3_M)

#define INT_BIT (unsigned)0x40 // (PD6) Interrupt Pin
#define INT_PCTL_M (unsigned)GPIO_PCTL_PD6_M
#define INT_PRIORITY 2

#define FSS_ADDR (*((volatile uint32_t *)(0x40007000 | (FSS_BIT << 2))))

#define READ(addr) (unsigned)(0x8000 | (addr << 8))
#define WRITE(addr, data) (unsigned)(0x7FFF & ((addr << 8) | data))

static uint16_t dmp;

// Defaults to USER_BANK_0
static USER_BANK LastUserBank = USER_BANK_0;

DELAY_FUNC IMU_Delay;

static uint32_t PD0_CLK;
static uint32_t PD1_FSS;
static uint32_t PD2_RX;
static uint32_t PD3_TX;
static uint32_t PD6_INT;

static void IMU_Interrupt_Init(void);
static void IMU_Config(void);
void GPIOD_Handler(void);

void GPIOD_Handler(void)
{
  uint16_t intStatus = 0;

  PD0_CLK = (GPIO_PORTD_DATA_R & 0x02) >> 0;
  PD1_FSS = (GPIO_PORTD_DATA_R & 0x02) >> 1;
  PD2_RX = (GPIO_PORTD_DATA_R & 0x04) >> 2;
  PD3_TX = (GPIO_PORTD_DATA_R & 0x08) >> 3;
  PD6_INT = (GPIO_PORTD_DATA_R & 0x40) >> 6;

  // Ensure the interrupt is on the INT pin and is a DMP interrupt
  if (GPIO_PORTD_MIS_R & INT_BIT)
  {
    intStatus = IMU_Read(INT_STATUS_ADDR);

    if (intStatus & DMP_INT)
    {
      // Get DMP data from FIFO register
      dmp = IMU_Read(FIFO_R_W_ADDR);
    }

    // Clear Interrupt
    GPIO_PORTD_ICR_R |= INT_BIT;
  }

  GPIO_PORTD_ICR_R |= CLK_BIT;
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
  GPIO_PORTD_IS_R &= ~(INT_BIT | CLK_BIT);

  // Only listen on one edge event
  GPIO_PORTD_IBE_R &= (~INT_BIT | CLK_BIT);

  // Trigger interrupt on Falling edge
  GPIO_PORTD_IEV_R &= ~INT_BIT;

  // Enable Port D's Interrupt Handler
  NVIC_EN0_R |= NVIC_EN0_INT3;

  // Configure Port D's priority
  NVIC_PRI0_R = (NVIC_PRI0_R & ~NVIC_PRI0_INT3_M) | (INT_PRIORITY << NVIC_PRI0_INT3_S);

  // Clear the INT pin's interrupt
  GPIO_PORTD_ICR_R |= INT_BIT | CLK_BIT;

  // Allow the INT pin interrupt to be detected
  GPIO_PORTD_IM_R |= INT_BIT | CLK_BIT;
}

static void IMU_Config(void)
{
  // Reset the device
  IMU_Write(PWR_MGMT_1_ADDR, DEVICE_RESET);

  // Wait 100ms after powering up
  IMU_Delay(100);

  // Wake the device from sleep, disable the Temp sensor, Turn off low power mode
  // and auto-selected clock source
  IMU_Write(PWR_MGMT_1_ADDR, CLKSEL_AUTO & ~(SLEEP_ENABLE | TEMP_ENABLE | LP_ENABLE));

  // Enable the Accelerometer and Gyroscope
  IMU_Write(PWR_MGMT_2_ADDR, ACCEL_ENABLE | GYRO_ENABLE);

  // Enable DMP and FIFO, and disable I2C for SPI only
  IMU_Write(USER_CTRL_ADDR, DMP_ENABLE | FIFO_ENABLE | SPI_ENABLE);

  // Configure gyro scale to 2000dps
  IMU_Write(GYRO_CONFIG_1_ADDR, GYRO_FS_SEL_2000);

  // Configure accelerometer scale to 16G
  IMU_Write(ACCEL_CONFIG_ADDR, ACCEL_FS_SEL_16G);

  // Specify the Interrupt pin is not open-drain (push-pull) and is an active high pin (falling edge interrupt)
  // also forces the Interrupt to be cleared for the level to be reset and any Read operation to clear the INT_STATUS
  IMU_Write(INT_PIN_CFG_ADDR, (INT_LATCH_MANUAL_CLEAR | INT_READ_CLEAR) & ~(INT_ACTIVE_LOW | INT_OPEN_DRAIN));

  // Enable DMP interrupt
  IMU_Write(INT_ENABLE_ADDR, DMP_INT_ENABLE);
}

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

  // Configure CLK, TX, and FSS as outputs
  GPIO_PORTD_DIR_R = (GPIO_PORTD_DIR_R & ~SPI_PINS) | CLK_BIT | TX_BIT | FSS_BIT;

  // Enable Digital Mode on pins
  GPIO_PORTD_DEN_R |= SPI_PINS;

  // Disable Analog Mode on pins
  GPIO_PORTD_AMSEL_R &= ~SPI_PINS;

  // Enable Pull-Up on Clock pin so is driven high when idle
  GPIO_PORTD_PUR_R |= CLK_BIT;

  // Disable SSI
  SSI3_CR1_R &= (unsigned)~SSI_CR1_SSE;
  // SSI3_CR1_R = (SSI3_CR1_R & (unsigned)~SSI_CR1_SSE) | SSI_CR1_LBM;

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
  SSI3_CR0_R = (ssiSCR << SSI_CR0_SCR_S) | SSI_CR0_FRF_MOTO | SSI_CR0_SPO | SSI_CR0_SPH | SSI_CR0_DSS_16;

  // Enable SSI
  SSI3_CR1_R |= SSI_CR1_SSE;

  // Force FSS pin high
  FSS_ADDR = FSS_BIT;
}

void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, DELAY_FUNC delay)
{
  if (delay == 0)
  {
    while (1)
      ;
  }

  IMU_Delay = delay;

  // Initialize the SPI pins
  SPI_Init(SYS_CLK, SSI_CLK);

  // Configure the IMU configuration settings
  IMU_Config();

  // Configure the Interrupt pin
  IMU_Interrupt_Init();
}

uint16_t IMU_Read(REG_ADDRESS REGISTER)
{
  uint8_t check, check2, check3;
  uint16_t pool = 0, busy_count = 0, no_data_count = 0;
  // Force FSS pin low
  FSS_ADDR = 0;

  if (LastUserBank != REGISTER.USER_BANK)
  {
    LastUserBank = REGISTER.USER_BANK;
    // Write to device the USER_BANK to read from
    SSI3_DR_R = WRITE(USER_BANK_ADDR.ADDRESS, REGISTER.USER_BANK);
  }

  // Put a 1 on the R/W bit to specify read
  SSI3_DR_R = READ(REGISTER.ADDRESS);

  // while (SSI3_SR_R & SSI_SR_BSY) {busy_count++;}

  // Force FSS pin high
  // Wait for Receive FIFO to not be empty
  while //(pool != 0xEA)
        // (no_data_count<8)
      (pool == 0x00)
  {
    check = SSI3_SR_R & SSI_SR_RNE;
    check2 = SSI3_SR_R & SSI_SR_TFE;
    check3 = SSI3_SR_R & SSI_SR_BSY;

    pool = (uint16_t)SSI3_DR_R;
    no_data_count++;
    if (check3)
      busy_count++;
  }

  FSS_ADDR = FSS_BIT;
  // Return data from register
  return pool;
}

void IMU_Write(REG_ADDRESS REGISTER, uint8_t data)
{
  FSS_ADDR = 0;

  if (LastUserBank != REGISTER.USER_BANK)
  {
    LastUserBank = REGISTER.USER_BANK;
    // Write to device the USER_BANK to read from
    SSI3_DR_R = WRITE(USER_BANK_ADDR.ADDRESS, REGISTER.USER_BANK);
  }

  // Put a 1 on the R/W bit to specify read
  SSI3_DR_R = WRITE(REGISTER.ADDRESS, data);

  FSS_ADDR = FSS_BIT;
}
