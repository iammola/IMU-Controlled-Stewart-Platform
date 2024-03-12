#include <stdint.h>

#include "tm4c123gh6pm.h"

#include "SSI0.h"

#define SSI0_CLK_BIT (1 << 2) // (PA2) SSI0CLK
#define SSI0_FSS_BIT (1 << 3) // (PA3) SSI0FSS (Chip Select)
#define SSI0_RX_BIT  (1 << 4) // (PA4) SSI0Rx
#define SSI0_TX_BIT  (1 << 5) // (PA5) SSI0Tx

#define SSI0_PINS      (unsigned)(SSI0_CLK_BIT | SSI0_FSS_BIT | SSI0_RX_BIT | SSI0_TX_BIT)
#define SSI0_PCTL      (unsigned)(GPIO_PCTL_PA2_SSI0CLK | GPIO_PCTL_PA3_SSI0FSS | GPIO_PCTL_PA4_SSI0RX | GPIO_PCTL_PA5_SSI0TX)
#define SSI0_PCTL_MASK (uint32_t)(GPIO_PCTL_PA2_M | GPIO_PCTL_PA3_M | GPIO_PCTL_PA4_M | GPIO_PCTL_PA5_M)

// Base of GPIO Port A Data Register
#define SSI0_FSS_ADDR (*((volatile uint32_t *)(0x40004000 | (SSI0_FSS_BIT << 2))))

#define WAIT_FOR_TX_SPACE()                                                                                                                          \
  while ((SSI0_SR_R & SSI_SR_TNF) != SSI_SR_TNF)                                                                                                     \
    ;
#define WAIT_FOR_RX_DATA()                                                                                                                           \
  while ((SSI0_SR_R & SSI_SR_RNE) != SSI_SR_RNE)                                                                                                     \
    ;
#define WAIT_FOR_IDLE()                                                                                                                              \
  while ((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY)                                                                                                     \
    ;

void SSI0_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, SSI_MODE frameConfig, DATA_SIZE dataSize) {
  uint32_t ssiSCR = 0;
  uint32_t ssiCPSR = 0;
  uint32_t maxBitRate = SYS_CLK / SSI_CLK;

  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; // Enable GPIO Port clock
  SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;   // Enable SSI module clock

  GPIO_PORTA_AFSEL_R |= SSI0_PINS;                                                                // Enable Alternate Functions on all pins
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~SSI0_PCTL_MASK) | SSI0_PCTL;                          // Enable SSI module peripheral functions
  GPIO_PORTA_DIR_R = (GPIO_PORTA_DIR_R & ~SSI0_PINS) | SSI0_CLK_BIT | SSI0_TX_BIT | SSI0_FSS_BIT; // Configure CLK, TX, and FSS as outputs
  GPIO_PORTA_DEN_R |= SSI0_PINS;                                                                  // Enable Digital Mode on pins
  GPIO_PORTA_AMSEL_R &= ~SSI0_PINS;                                                               // Disable Analog Mode on pins

  if (frameConfig & SSI_CR0_SPO)
    GPIO_PORTA_PUR_R |= SSI0_CLK_BIT; // Enable Pull-Up on Clock pin so is driven high when idle
  else
    GPIO_PORTA_PDR_R |= SSI0_CLK_BIT; // Enable Pull-Down on Clock pin so is driven low when idle

  SSI0_CR1_R &= (unsigned)~SSI_CR1_SSE; // Disable SSI
  SSI0_CR0_R &= (unsigned)~SSI_CR1_MS;  // Enable Master mode
  SSI0_CC_R = SSI_CC_CS_SYSPLL;         // Configure for system clock

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

  SSI0_CPSR_R = ssiCPSR; // Set the calculated pre-scale divisor
  // Set the calculated clock rate, and the specified frame config and data size
  SSI0_CR0_R = (ssiSCR << SSI_CR0_SCR_S) | (frameConfig & ~(SSI_CR0_SCR_M | SSI_CR0_DSS_M)) | (dataSize & SSI_CR0_DSS_M);
  SSI0_CR1_R |= SSI_CR1_SSE; // Enable SSI

  // Force FSS pin high
  SSI0_FSS_ADDR = SSI0_FSS_BIT;
}

void SSI0_Read(uint16_t initData, uint16_t *result, uint32_t length) {
  uint32_t __stale = 0;

  // Start transmission
  SSI0_FSS_ADDR = 0;

  // Clear RX FIFO
  while ((SSI0_SR_R & SSI_SR_RNE) == SSI_SR_RNE) {
    __stale = SSI0_DR_R;
  }

  WAIT_FOR_TX_SPACE() // Wait for space in TX FIFO

  // Get requested amount of data
  while (length > 0) {
    SSI0_DR_R = initData; // Set data to initiate read

    WAIT_FOR_RX_DATA() // Wait for data to be stored in FIFO.

    *result = (uint16_t)SSI0_DR_R; // store data

    // increment pointer
    if (--length > 0)
      result++;
  }
}

void SSI0_Write(uint16_t *data, uint32_t length) {
  while (length > 0) {
    WAIT_FOR_TX_SPACE() // Prevent setting data in full TX FIFO

    SSI0_DR_R = *data; // set data to be transmitted

    // increment pointer
    if (--length > 0)
      data++;
  }

  WAIT_FOR_IDLE() // Wait for Idle before ending transmission
}

void SSI0_StartTransmission(void) {
  SSI0_FSS_ADDR = 0; // Start transmission
}

void SSI0_EndTransmission(void) {
  SSI0_FSS_ADDR = SSI0_FSS_BIT; // End transmission
}
