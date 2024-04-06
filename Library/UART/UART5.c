/**
 * @file UART5.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-14
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <stdbool.h>
#include <stdint.h>

#include "UART5.h"

#include "tm4c123gh6pm.h"

#define UART5_PINS      (unsigned)(UART5_RX_BIT | UART5_TX_BIT)
#define UART5_PCTL_MASK (unsigned)(GPIO_PCTL_PE4_M | GPIO_PCTL_PE5_M)
#define UART5_PCTL      (unsigned)(GPIO_PCTL_PE4_U5RX | GPIO_PCTL_PE5_U5TX)

static void UART5_NVIC_Enable(uint8_t interruptPriority);
static void UART5_BRDConfigure(uint32_t SYS_CLOCK, uint32_t baudRate);
static void UART5_LCRHConfigure(uint8_t wordLength, uint8_t parity, bool useTwoStopBits);

static void UART5_BRDConfigure(uint32_t SYS_CLOCK, uint32_t baudRate) {
  // High speed mode must be used if the BAUD rate would run faster than the system clock
  bool needsHighSpeed = (baudRate * 16) > SYS_CLOCK;

  // Basically the Baud-Rate Generation formula given, BRD = SYS_CLOCK / (CLK_DIV * BAUD_RATE);
  // But uses Fixed Point Arithmetic by multiplying the values by specific power of 2
  // Depending on the need for the High Speed Mode, the CLK_DIV changes
  uint32_t BRD = ((SYS_CLOCK / (needsHighSpeed ? 8 : 16)) << FRACTIONAL_BRD_MULTIPLIER) / baudRate;

  // Dividing it by the initial multiplier would then give us back the integer value
  uint32_t integerPart = BRD >> FRACTIONAL_BRD_MULTIPLIER;

  // The fractional part would be stored in the number of LSBs shifted by the multiplier. Therefore the
  // multiplication would not be required again. All that'll be left to do will be to filter that part out
  // using a mask with all the bits it was shifted by as ones (1). i.e (2 ^ multiplier) - 1.
  // The value would be a number between 0 and 2^multiplier. Where 0 would be 0.0, half of 2^M would be 0.5, and vice versa.
  uint32_t fractionalPart = (uint32_t)((BRD & FRACTIONAL_BRD_MASK) + 0.5);

  UART5_IBRD_R = integerPart;    // Write calculated BRD integer part
  UART5_FBRD_R = fractionalPart; // Write calculated fractional part

  if (needsHighSpeed)
    UART5_CTL_R |= UART_CTL_HSE; // Enable High-Speed Mode
  else
    UART5_CTL_R &= (unsigned)~UART_CTL_HSE; // Disable High-Speed Mode
}

static void UART5_LCRHConfigure(uint8_t wordLength, uint8_t parity, bool useTwoStopBits) {
  // Enable FIFO buffers
  uint32_t result = UART_LCRH_FEN;

  // Specify word length, falling back to 5 bits.
  switch (wordLength) {
    case WORD_8_BIT:
      result |= UART_LCRH_WLEN_8;
      break;
    case WORD_7_BIT:
      result |= UART_LCRH_WLEN_7;
      break;
    case WORD_6_BIT:
      result |= UART_LCRH_WLEN_6;
      break;
    default:
      result |= UART_LCRH_WLEN_5;
      break;
  }

  switch (parity) {
    case EVEN_PARITY:
      result |= UART_LCRH_PEN | UART_LCRH_EPS; // Enable Even Parity
      break;
    case ODD_PARITY:
      result |= UART_LCRH_PEN;
      break;
    default:
      break;
  }

  if (useTwoStopBits)
    result |= UART_LCRH_STP2; // Enable Two Stop Bits

  UART5_LCRH_R = result;
}

static void UART5_NVIC_Enable(uint8_t interruptPriority) {
  NVIC_EN1_R |= NVIC_EN0_INT29;                                                                            // Enable Interrupt 61 for UART 5
  NVIC_PRI15_R = (NVIC_PRI15_R & (unsigned)~NVIC_PRI15_INTB_M) | (interruptPriority << NVIC_PRI15_INTB_S); // Set Priority
}

void UART5_Enable(void) {
  UART5_CTL_R |= UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
}

void UART5_Disable(void) {
  // Wait for transmission to finish
  while (UART5_FR_R & UART_FR_BUSY)
    ;

  UART5_LCRH_R &= (unsigned)~UART_LCRH_FEN;                                  // Clear the FIFO
  UART5_CTL_R &= (unsigned)~(UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE); // Disable the UART
}

void UART5_TimeoutInterrupt(uint8_t interruptPriority) {
  UART5_IM_R |= UART_IM_RTIM; // Receive Timeout IRQ
  UART5_NVIC_Enable(interruptPriority);
}

void UART5_FIFOInterrupt(uint8_t RXFIFOLevel, uint8_t interruptPriority) {
  UART5_IM_R |= UART_IM_RXIM; // Receive FIFO Level IRQ

  // Set RX Interrupt Levels
  switch (RXFIFOLevel) {
    case RX_FIFO_7_8:
      RXFIFOLevel = UART_IFLS_RX7_8;
      break;
    case RX_FIFO_6_8:
      RXFIFOLevel = UART_IFLS_RX6_8;
      break;
    case RX_FIFO_2_8:
      RXFIFOLevel = UART_IFLS_RX2_8;
      break;
    case RX_FIFO_1_8:
      RXFIFOLevel = UART_IFLS_RX1_8;
      break;
    default:
      RXFIFOLevel = UART_IFLS_RX4_8;
      break;
  }

  // Set RX FIFO level
  UART5_IFLS_R = (UART5_IFLS_R & (unsigned)~UART_IFLS_RX_M) | RXFIFOLevel;

  UART5_NVIC_Enable(interruptPriority);
}

void UART5_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t parity, bool useTwoStopBits) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // Enable Port E's clock
  SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R5; // Enable UART module 5's clock

  GPIO_PORTE_AFSEL_R |= UART5_PINS; // Enable Alternate functions in PINS.
  GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & ~UART5_PCTL_MASK) |
                      UART5_PCTL; // Enable UART Tx and Rx functions by masking the pin peripherals byte and setting the UART value
  GPIO_PORTE_DIR_R = (GPIO_PORTE_DIR_R & ~UART5_RX_BIT) | UART5_TX_BIT; // Configure Rx as an input and Tx as an output
  GPIO_PORTE_DEN_R |= UART5_PINS;                                       // Enable Digital on PINS.
  GPIO_PORTE_AMSEL_R &= ~UART5_PINS;                                    // Disable Analog on PINS.

  UART5_Disable(); // Disable UART

  UART5_BRDConfigure(SYS_CLOCK, baudRate);                 // Set Baud-Rate Divisor (BRD)
  UART5_LCRHConfigure(wordLength, useTwoStopBits, parity); // Configure (Line Control) LCRH

  UART5_Enable(); // Enable UART
}

void UART5_Transmit(uint8_t *data, uint32_t byteCount) {
  uint8_t byteIndex = 0;

  do {
    // Prevent Data from being set while the transmit FIFO is full
    while (UART5_FR_R & UART_FR_TXFF)
      ;

    // Transmit data byte
    UART5_DR_R = data[byteIndex];

    // Increment index tracker
    byteIndex++;
  } while (byteIndex < byteCount);
}

void UART5_Receive(uint8_t *data, uint32_t length, uint32_t maxTicks) {
  volatile uint32_t ticks = 0x00;

  while (length > 0) {
    ticks = maxTicks;

    while ((UART5_FR_R & UART_FR_RXFE) && --ticks > 0) { // Wait for Receive FIFO to have data
    }

    *data = (uint8_t)UART5_DR_R; // Read data

    if (--length > 0) // Increment pointer if there's still data to write
      data++;
  }
}
