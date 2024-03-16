/**
 * @file UART1.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <stdbool.h>
#include <stdint.h>

#include "UART1.h"

#include "tm4c123gh6pm.h"

#define UART1_RX_BIT (unsigned)(1 << 4) // PB0 - U1RX
#define UART1_TX_BIT (unsigned)(1 << 5) // PB1 - U1TX

#define UART1_PINS      (unsigned)(UART1_RX_BIT | UART1_TX_BIT)
#define UART1_PCTL_MASK (unsigned)(GPIO_PCTL_PB0_M | GPIO_PCTL_PB1_M)
#define UART1_PCTL      (unsigned)(GPIO_PCTL_PB0_U1RX | GPIO_PCTL_PB1_U1TX)

static void UART1_NVIC_Enable(uint8_t interruptPriority);
static void UART1_BRDConfigure(uint32_t SYS_CLOCK, uint32_t baudRate);
static void UART1_LCRHConfigure(uint8_t wordLength, uint8_t parity, bool useTwoStopBits);

static void UART1_BRDConfigure(uint32_t SYS_CLOCK, uint32_t baudRate) {
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

  UART1_IBRD_R = integerPart;    // Write calculated BRD integer part
  UART1_FBRD_R = fractionalPart; // Write calculated fractional part

  if (needsHighSpeed)
    UART1_CTL_R |= UART_CTL_HSE; // Enable High-Speed Mode
  else
    UART1_CTL_R &= (unsigned)~UART_CTL_HSE; // Disable High-Speed Mode
}

static void UART1_LCRHConfigure(uint8_t wordLength, uint8_t parity, bool useTwoStopBits) {
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

  UART1_LCRH_R = result;
}

static void UART1_NVIC_Enable(uint8_t interruptPriority) {
  NVIC_EN1_R |= NVIC_EN0_INT29;                                                                            // Enable Interrupt 61 for UART 5
  NVIC_PRI15_R = (NVIC_PRI15_R & (unsigned)~NVIC_PRI15_INTB_M) | (interruptPriority << NVIC_PRI15_INTB_S); // Set Priority
}

void UART1_Enable(void) {
  UART1_CTL_R |= UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
}

void UART1_Disable(void) {
  // Wait for transmission to finish
  while (UART1_FR_R & UART_FR_BUSY)
    ;

  UART1_LCRH_R &= (unsigned)~UART_LCRH_FEN;                                  // Clear the FIFO
  UART1_CTL_R &= (unsigned)~(UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE); // Disable the UART
}

void UART1_TimeoutInterrupt(uint8_t interruptPriority) {
  UART1_IM_R |= UART_IM_RTIM; // Receive Timeout IRQ
  UART1_NVIC_Enable(interruptPriority);
}

void UART1_FIFOInterrupt(uint8_t RXFIFOLevel, uint8_t interruptPriority) {
  UART1_IM_R |= UART_IM_RXIM; // Receive FIFO Level IRQ

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
  UART1_IFLS_R = (UART1_IFLS_R & (unsigned)~UART_IFLS_RX_M) | RXFIFOLevel;

  UART1_NVIC_Enable(interruptPriority);
}

void UART1_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t parity, bool useTwoStopBits) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // Enable Port B's clock
  SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1; // Enable UART module 1's clock

  GPIO_PORTB_AFSEL_R |= UART1_PINS; // Enable Alternate functions in PINS 4 and 5.
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~UART1_PCTL_MASK) |
                      UART1_PCTL; // Enable UART Tx and Rx functions by masking the pin peripherals byte and setting the UART value
  GPIO_PORTB_DIR_R = (GPIO_PORTB_DIR_R & ~UART1_RX_BIT) | UART1_TX_BIT; // Configure Pin 4 (UART1Rx) as an input and Pin 5 (UART1Tx) as an output
  GPIO_PORTB_DEN_R |= UART1_PINS;                                       // Enable Digital on PINS 4 and 5.
  GPIO_PORTB_AMSEL_R &= ~UART1_PINS;                                    // Disable Analog on PINS 4 and 5.

  UART1_Disable(); // Disable UART

  UART1_BRDConfigure(SYS_CLOCK, baudRate);                 // Set Baud-Rate Divisor (BRD)
  UART1_LCRHConfigure(wordLength, useTwoStopBits, parity); // Configure (Line Control) LCRH

  UART1_Enable(); // Enable UART
}

void UART1_Transmit(uint8_t *data, uint32_t byteCount) {
  uint8_t byteIndex = 0;

  do {
    // Prevent Data from being set while the transmit FIFO is full
    while (UART1_FR_R & UART_FR_TXFF)
      ;

    // Transmit data byte
    UART1_DR_R = data[byteIndex];

    // Increment index tracker
    byteIndex++;
  } while (byteIndex < byteCount);
}

bool UART1_Receive(uint8_t *data, uint32_t length) {
  uint16_t wait = 0;

  while (length > 0) {
    for (wait = 0xFFFF; (wait > 0) && (UART1_FR_R & UART_FR_RXFE); wait--) {
    } // Wait for Receive FIFO to have data

    *data = (uint8_t)UART1_DR_R; // Read data

    if (--length > 0) // Increment pointer if there's still data to write
      data++;
  }

  return true;
}
