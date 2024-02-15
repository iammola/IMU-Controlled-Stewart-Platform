#include <stdint.h>

#include "tm4c123gh6pm.h"

#include "SPI.h"

#define SSI2_CLK_BIT 1 << 4 // (PB4) SSI2CLK
#define SSI2_FSS_BIT 1 << 5 // (PB5) SSI2FSS (Chip Select)
#define SSI2_RX_BIT  1 << 6 // (PB6) SSI2Rx
#define SSI2_TX_BIT  1 << 7 // (PB7) SSI2Tx

#define SSI2_PINS      (unsigned)(SSI2_CLK_BIT | SSI2_FSS_BIT | SSI2_RX_BIT | SSI2_TX_BIT)
#define SSI2_PCTL      (unsigned)(GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB5_SSI2FSS | GPIO_PCTL_PB6_SSI2RX | GPIO_PCTL_PB7_SSI2TX)
#define SSI2_PCTL_MASK (uint32_t)(GPIO_PCTL_PB4_M | GPIO_PCTL_PB5_M | GPIO_PCTL_PB6_M | GPIO_PCTL_PB7_M)

#define SSI2_FSS_ADDR   (*((volatile uint32_t *)(0x40005000 | (SSI2_FSS_BIT << 2))))
#define MAX_SPI_RX_WAIT 8

#define WAIT_FOR_TX_SPACE()                                                                                                                          \
  while ((SSI2_SR_R & SSI_SR_TNF) != SSI_SR_TNF)                                                                                                     \
    ;
#define WAIT_FOR_RX_DATA(data)                                                                                                                       \
  for (data = 0; (data < MAX_SPI_RX_WAIT) && (SSI2_SR_R & SSI_SR_RNE) != SSI_SR_RNE; data++)                                                         \
    ;
#define WAIT_FOR_IDLE()                                                                                                                              \
  while ((SSI2_SR_R & SSI_SR_BSY) == SSI_SR_BSY)                                                                                                     \
    ;

void SPI2_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, uint8_t frameConfig, uint8_t dataSize) {
  uint32_t ssiSCR = 0;
  uint32_t ssiCPSR = 0;
  uint32_t maxBitRate = SYS_CLK / SSI_CLK;

  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // Enable Port B clock
  SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;   // Enable SSI module 2 clock

  GPIO_PORTB_AFSEL_R |= SSI2_PINS;                                                                // Enable Alternate Functions on all pins
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~SSI2_PCTL_MASK) | SSI2_PCTL;                          // Enable SSI module 3 peripheral functions
  GPIO_PORTB_DIR_R = (GPIO_PORTB_DIR_R & ~SSI2_PINS) | SSI2_CLK_BIT | SSI2_TX_BIT | SSI2_FSS_BIT; // Configure CLK, TX, and FSS as outputs
  GPIO_PORTB_DEN_R |= SSI2_PINS;                                                                  // Enable Digital Mode on pins
  GPIO_PORTB_AMSEL_R &= ~SSI2_PINS;                                                               // Disable Analog Mode on pins

  if (frameConfig & SSI_CR0_SPO)
    GPIO_PORTB_PUR_R |= SSI2_CLK_BIT; // Enable Pull-Up on Clock pin so is driven high when idle
  else
    GPIO_PORTB_PDR_R |= SSI2_CLK_BIT; // Enable Pull-Down on Clock pin so is driven low when idle

  SSI2_CR1_R &= (unsigned)~SSI_CR1_SSE; // Disable SSI

  SSI2_CR0_R &= (unsigned)~SSI_CR1_MS; // Enable Master mode
  SSI2_CC_R = SSI_CC_CS_SYSPLL;        // Configure for system clock

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

  // The SSI Clk frequency needs to be increased
  if (ssiCPSR > 254) {
    while (1)
      ;
  }

  // Set the calculated clock rate, and the specified frame config and data size
  SSI2_CR0_R = (ssiSCR << SSI_CR0_SCR_S) | (frameConfig & ~(SSI_CR0_SCR_M | SSI_CR0_DSS_M)) | (dataSize & SSI_CR0_DSS_M);
  SSI2_CPSR_R = ssiCPSR;     // Set the calculated pre-scale divisor
  SSI2_CR1_R |= SSI_CR1_SSE; // Enable SSI

  SSI2_FSS_ADDR = SSI2_FSS_BIT; // Force FSS pin high
}

void SPI2_Read(uint16_t initData, uint16_t *result, uint8_t length) {
  uint8_t  wait = 0;
  uint32_t __stale = 0;

  while ((SSI2_SR_R & SSI_SR_RNE) == SSI_SR_RNE) {
    __stale = SSI2_DR_R; // Clear RX FIFO
  }

  do {
    WAIT_FOR_TX_SPACE()   // Wait for space in TX FIFO
    SSI2_DR_R = initData; // Set data to initiate read

    WAIT_FOR_RX_DATA(wait) // Wait for data to be stored in FIFO

    if (SSI2_SR_R & SSI_SR_RNE)
      break;
  } while (wait > 0);

  // Get requested amount of data
  while (length > 0) {
    WAIT_FOR_RX_DATA(wait) // Wait for data to be stored in FIFO

    *result = (uint16_t)SSI2_DR_R; // read data

    if (--length > 0)
      result++; // increment pointer
  }

  WAIT_FOR_IDLE() // Wait for Idle before ending transmission
}

void SPI2_Write(uint16_t *data, uint8_t length) {
  uint32_t __stale = 0;

  // Send desired amount of data
  while (length > 0) {
    WAIT_FOR_TX_SPACE() // Prevent setting data in full TX FIFO
    SSI2_DR_R = *data;  // set data to be transmitted

    if (--length > 0)
      data++; // increment pointer
  }

  WAIT_FOR_IDLE() // Wait for Idle before ending transmission

  while ((SSI2_SR_R & SSI_SR_RNE) == SSI_SR_RNE) {
    __stale = SSI2_DR_R; // Clear RX FIFO
  }
}

void SPI2_StartTransmission(void) {
  SSI2_FSS_ADDR = 0; // Start transmission
}

void SPI2_EndTransmission(void) {
  SSI2_FSS_ADDR = SSI2_FSS_BIT; // End transmission
}
