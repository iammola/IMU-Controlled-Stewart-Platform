#include <stdint.h>

#include "tm4c123gh6pm.h"

#include "SPI.h"

#define SSI1_CLK_BIT (1 << 2) // (PD0) SSI1CLK
#define SSI1_FSS_BIT (1 << 3) // (PD1) SSI1FSS (Chip Select)
#define SSI1_RX_BIT  (1 << 4) // (PD2) SSI1Rx
#define SSI1_TX_BIT  (1 << 5) // (PD3) SSI1Tx

#define SSI1_PINS      (unsigned)(SSI1_CLK_BIT | SSI1_FSS_BIT | SSI1_RX_BIT | SSI1_TX_BIT)
#define SSI1_PCTL      (unsigned)(GPIO_PCTL_PD0_SSI1CLK | GPIO_PCTL_PD1_SSI1FSS | GPIO_PCTL_PD2_SSI1RX | GPIO_PCTL_PD3_SSI1TX)
#define SSI1_PCTL_MASK (uint32_t)(GPIO_PCTL_PD0_M | GPIO_PCTL_PD1_M | GPIO_PCTL_PD2_M | GPIO_PCTL_PD3_M)

// Base of GPIO Port D Data Register
#define SSI1_FSS_ADDR (*((volatile uint32_t *)(0x40007000 | (SSI1_FSS_BIT << 2))))

#define WAIT_FOR_TX_SPACE()                                                                                                                          \
  while ((SSI1_SR_R & SSI_SR_TNF) != SSI_SR_TNF)                                                                                                     \
    ;
#define WAIT_FOR_RX_DATA()                                                                                                                           \
  while ((SSI1_SR_R & SSI_SR_RNE) != SSI_SR_RNE)                                                                                                     \
    ;
#define WAIT_FOR_IDLE()                                                                                                                              \
  while ((SSI1_SR_R & SSI_SR_BSY) == SSI_SR_BSY)                                                                                                     \
    ;

void SPI1_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, uint8_t frameConfig, uint8_t dataSize) {
  uint32_t ssiSCR = 0;
  uint32_t ssiCPSR = 0;
  uint32_t maxBitRate = SYS_CLK / SSI_CLK;

  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // Enable Port D clock
  SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1;   // Enable SSI module 1 clock

  GPIO_PORTD_AFSEL_R |= SSI1_PINS;                                                                // Enable Alternate Functions on all pins
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & ~SSI1_PCTL_MASK) | SSI1_PCTL;                          // Enable SSI module 1 peripheral functions
  GPIO_PORTD_DIR_R = (GPIO_PORTD_DIR_R & ~SSI1_PINS) | SSI1_CLK_BIT | SSI1_TX_BIT | SSI1_FSS_BIT; // Configure CLK, TX, and FSS as outputs
  GPIO_PORTD_DEN_R |= SSI1_PINS;                                                                  // Enable Digital Mode on pins
  GPIO_PORTD_AMSEL_R &= ~SSI1_PINS;                                                               // Disable Analog Mode on pins

  if (frameConfig & SSI_CR0_SPO)
    GPIO_PORTD_PUR_R |= SSI1_CLK_BIT; // Enable Pull-Up on Clock pin so is driven high when idle
  else
    GPIO_PORTD_PDR_R |= SSI1_CLK_BIT; // Enable Pull-Down on Clock pin so is driven low when idle

  SSI1_CR1_R &= (unsigned)~SSI_CR1_SSE; // Disable SSI
  SSI1_CR0_R &= (unsigned)~SSI_CR1_MS;  // Enable Master mode
  SSI1_CC_R = SSI_CC_CS_SYSPLL;         // Configure for system clock

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

  SSI1_CPSR_R = ssiCPSR; // Set the calculated pre-scale divisor
  // Set the calculated clock rate, and the specified frame config and data size
  SSI1_CR0_R = (ssiSCR << SSI_CR0_SCR_S) | (frameConfig & ~(SSI_CR0_SCR_M | SSI_CR0_DSS_M)) | (dataSize & SSI_CR0_DSS_M);
  SSI1_CR1_R |= SSI_CR1_SSE; // Enable SSI

  // Force FSS pin high
  SSI1_FSS_ADDR = SSI1_FSS_BIT;
}

void SPI1_Read(uint16_t initData, uint16_t *result, uint32_t length) {
  uint32_t __stale = 0;

  // Start transmission
  SSI1_FSS_ADDR = 0;

  // Clear RX FIFO
  while ((SSI1_SR_R & SSI_SR_RNE) == SSI_SR_RNE) {
    __stale = SSI1_DR_R;
  }

  WAIT_FOR_TX_SPACE() // Wait for space in TX FIFO

  SSI1_DR_R = initData; // Set data to initiate read

  // Get requested amount of data
  while (length > 0) {
    WAIT_FOR_RX_DATA() // Wait for data to be stored in FIFO.

    *result = (uint16_t)SSI1_DR_R; // store data

    // increment pointer
    if (--length > 0)
      result++;
  }
}

void SPI1_Write(uint16_t *data, uint32_t length) {
  while (length > 0) {
    WAIT_FOR_TX_SPACE() // Prevent setting data in full TX FIFO

    SSI1_DR_R = *data; // set data to be transmitted

    // increment pointer
    if (--length > 0)
      data++;
  }

  WAIT_FOR_IDLE() // Wait for Idle before ending transmission
}

void SPI1_StartTransmission(void) {
  SSI1_FSS_ADDR = 0; // Start transmission
}

void SPI1_EndTransmission(void) {
  SSI1_FSS_ADDR = SSI1_FSS_BIT; // End transmission
}
