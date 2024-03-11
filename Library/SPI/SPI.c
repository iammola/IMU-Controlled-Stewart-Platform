#include <stdint.h>

#include "tm4c123gh6pm.h"

#include "SPI.h"

#if SSI_MODULE == 0
#define CLK_BIT (1 << 2) // (PA2) SSI0CLK
#define FSS_BIT (1 << 3) // (PA3) SSI0FSS (Chip Select)
#define RX_BIT  (1 << 4) // (PA4) SSI0Rx
#define TX_BIT  (1 << 5) // (PA5) SSI0Tx

#define PCTL      (unsigned)(GPIO_PCTL_PA2_SSI0CLK | GPIO_PCTL_PA3_SSI0FSS | GPIO_PCTL_PA4_SSI0RX | GPIO_PCTL_PA5_SSI0TX)
#define PCTL_MASK (uint32_t)(GPIO_PCTL_PA2_M | GPIO_PCTL_PA3_M | GPIO_PCTL_PA4_M | GPIO_PCTL_PA5_M)

#define PORT_BASE 0x40004000
#define SSI_BASE  0x40008000

#define SPI_MODULE_ENABLE   SYSCTL_RCGCSSI_R0
#define CLOCK_MODULE_ENABLE SYSCTL_RCGCGPIO_R0
#elif SSI_MODULE == 1
#define CLK_BIT (1 << 0) // (PD0) SSI1CLK
#define FSS_BIT (1 << 1) // (PD1) SSI1FSS (Chip Select)
#define RX_BIT  (1 << 2) // (PD2) SSI1Rx
#define TX_BIT  (1 << 3) // (PD3) SSI1Tx

#define PCTL      (unsigned)(GPIO_PCTL_PD0_SSI1CLK | GPIO_PCTL_PD1_SSI1FSS | GPIO_PCTL_PD2_SSI1RX | GPIO_PCTL_PD3_SSI1TX)
#define PCTL_MASK (uint32_t)(GPIO_PCTL_PD0_M | GPIO_PCTL_PD1_M | GPIO_PCTL_PD2_M | GPIO_PCTL_PD3_M)

#define PORT_BASE 0x40007000
#define SSI_BASE  0x40009000

#define SPI_MODULE_ENABLE   SYSCTL_RCGCSSI_R1
#define CLOCK_MODULE_ENABLE SYSCTL_RCGCGPIO_R3
#endif

#define PINS (unsigned)(CLK_BIT | FSS_BIT | RX_BIT | TX_BIT)

#define FSS_ADDR (*((volatile uint32_t *)(PORT_BASE | (FSS_BIT << 2)))) // GPIO Port, Pin Data Register

#define GPIO_DIR_R   (*((volatile uint32_t *)(PORT_BASE | 0x400)))
#define GPIO_AFSEL_R (*((volatile uint32_t *)(PORT_BASE | 0x420)))
#define GPIO_DEN_R   (*((volatile uint32_t *)(PORT_BASE | 0x51C)))
#define GPIO_PUR_R   (*((volatile uint32_t *)(PORT_BASE | 0x510)))
#define GPIO_PDR_R   (*((volatile uint32_t *)(PORT_BASE | 0x514)))
#define GPIO_AMSEL_R (*((volatile uint32_t *)(PORT_BASE | 0x528)))
#define GPIO_PCTL_R  (*((volatile uint32_t *)(PORT_BASE | 0x52C)))

#define SSI_CR0_R  (*((volatile uint32_t *)(SSI_BASE | 0x000)))
#define SSI_CR1_R  (*((volatile uint32_t *)(SSI_BASE | 0x004)))
#define SSI_DR_R   (*((volatile uint32_t *)(SSI_BASE | 0x008)))
#define SSI_SR_R   (*((volatile uint32_t *)(SSI_BASE | 0x00C)))
#define SSI_CPSR_R (*((volatile uint32_t *)(SSI_BASE | 0x010)))
#define SSI_CC_R   (*((volatile uint32_t *)(SSI_BASE | 0xFC8)))

#define WAIT_FOR_TX_SPACE()                                                                                                                          \
  while ((SSI_SR_R & SSI_SR_TNF) != SSI_SR_TNF)                                                                                                      \
    ;
#define WAIT_FOR_RX_DATA()                                                                                                                           \
  while ((SSI_SR_R & SSI_SR_RNE) != SSI_SR_RNE)                                                                                                      \
    ;
#define WAIT_FOR_IDLE()                                                                                                                              \
  while ((SSI_SR_R & SSI_SR_BSY) == SSI_SR_BSY)                                                                                                      \
    ;
#define CLEAR_RX_FIFO()                                                                                                                              \
  while ((SSI_SR_R & SSI_SR_RNE) == SSI_SR_RNE)                                                                                                      \
    __stale = SSI_DR_R;

void SPI_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, SPI_MODE frameConfig, DATA_SIZE dataSize) {
  uint32_t ssiSCR = 0;
  uint32_t ssiCPSR = 0;
  uint32_t maxBitRate = SYS_CLK / SSI_CLK;

  SYSCTL_RCGCGPIO_R |= CLOCK_MODULE_ENABLE; // Enable Port clock
  SYSCTL_RCGCSSI_R |= SPI_MODULE_ENABLE;    // Enable SPI Module Clock

  GPIO_AFSEL_R |= PINS;                                           // Enable Alternate Functions on all pins
  GPIO_PCTL_R = (GPIO_PCTL_R & ~PCTL_MASK) | PCTL;                // Enable SSI module 0 peripheral functions
  GPIO_DIR_R = (GPIO_DIR_R & ~PINS) | CLK_BIT | TX_BIT | FSS_BIT; // Configure CLK, TX, and FSS as outputs
  GPIO_DEN_R |= PINS;                                             // Enable Digital Mode on pins
  GPIO_AMSEL_R &= ~PINS;                                          // Disable Analog Mode on pins

  if (frameConfig & SSI_CR0_SPO)
    GPIO_PUR_R |= CLK_BIT; // Enable Pull-Up on Clock pin so is driven high when idle
  else
    GPIO_PDR_R |= CLK_BIT; // Enable Pull-Down on Clock pin so is driven low when idle

  SSI_CR1_R &= (unsigned)~SSI_CR1_SSE; // Disable SSI
  SSI_CR0_R &= (unsigned)~SSI_CR1_MS;  // Enable Master mode
  SSI_CC_R = SSI_CC_CS_SYSPLL;         // Configure for system clock

  // As a master, the system clock must be 2 times faster than the SSI_CLK and the
  // SSI_CLK cannot be more than 25MHz
  if ((SYS_CLK < (SSI_CLK * 2)) || SSI_CLK > 25e6) {
    while (1)
      ;
  }

  do {
    ssiCPSR += 2;                        // Increment Pre-scale Divisor by 2
    ssiSCR = (maxBitRate / ssiCPSR) - 1; // Calculate new clock rate
  } while (ssiSCR > 255);

  if (ssiCPSR > 254) { // The SSI Clk frequency needs to be increased
    while (1)
      ;
  }

  SSI_CPSR_R = ssiCPSR; // Set the calculated pre-scale divisor
  // Set the calculated clock rate, and the specified frame config and data size
  SSI_CR0_R = (ssiSCR << SSI_CR0_SCR_S) | (frameConfig & ~(SSI_CR0_SCR_M | SSI_CR0_DSS_M)) | (dataSize & SSI_CR0_DSS_M);
  SSI_CR1_R |= SSI_CR1_SSE; // Enable SSI

  // Force FSS pin high
  FSS_ADDR = FSS_BIT;
}

void SPI_Read(uint16_t initData, uint16_t *result, uint32_t length) {
  uint32_t __stale = 0;

  // Start transmission
  FSS_ADDR = 0;

  CLEAR_RX_FIFO()
  WAIT_FOR_TX_SPACE() // Wait for space in TX FIFO

  // Get requested amount of data
  while (length > 0) {
    SSI_DR_R = initData; // Set data to initiate read
    WAIT_FOR_RX_DATA()   // Wait for data to be stored in FIFO.

    *result = (uint16_t)SSI_DR_R; // store data

    // increment pointer
    if (--length > 0)
      result++;
  }
}

void SPI_Write(uint16_t *data, uint32_t length) {
  while (length > 0) {
    WAIT_FOR_TX_SPACE() // Prevent setting data in full TX FIFO

    SSI_DR_R = *data; // set data to be transmitted

    // increment pointer
    if (--length > 0)
      data++;
  }

  WAIT_FOR_IDLE() // Wait for Idle before ending transmission
}

void SPI_StartTransmission(void) {
  FSS_ADDR = 0; // Start transmission
}

void SPI_EndTransmission(void) {
  FSS_ADDR = FSS_BIT; // End transmission
}
