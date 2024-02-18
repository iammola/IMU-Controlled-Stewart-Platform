/**
 * Name: Ademola Adedeji
 * Date: 27th January, 2024
 * Course-Code: ELTR-2400
 * Course-Name: Communication Systems II
 *
 * Lab Description: Initializes UART module 4 on Port C pins 4 (Tx) and 5 (Tx).
 *                  Automatically calculates the need for High Speed Mode (HSE),
 *                  and the Integer and Fractional parts of the Baud Rate. It allows
 *                  for configuration of the Data length, Use of Parity and number of
 *                  stop bits. Only uses Receive interrupts, and also allows for configureaiton
 *                  of the FIFO level to trigger the Receive FIFO interrupt.
 */
#include "UART.h"

#include <stdbool.h>
#include <stdint.h>

#include "tm4c123gh6pm.h"

#define RX_BIT (unsigned)(1 << 0) // PB0
#define TX_BIT (unsigned)(1 << 1) // PB1

#define PCTL_MASK (unsigned)(GPIO_PCTL_PB0_M | GPIO_PCTL_PB1_M)
#define PCTL      (unsigned)(GPIO_PCTL_PB0_U1RX | GPIO_PCTL_PB1_U1TX)

#define FRACTIONAL_BRD_MULTIPLIER 6 // 2^6 = 64 or LSH 6 times
#define FRACTIONAL_BRD_MASK       (1 << FRACTIONAL_BRD_MULTIPLIER) - 1

/* Calculates and sets the Integer and Fractional of the BRD and HSE */
static void UART_BRDConfigure(uint32_t SYS_CLOCK, uint32_t baudRate);
/* Sets the LCRH configuration */
static void UART_LCRHConfigure(uint8_t wordLength, uint8_t parity, bool useTwoStopBits);

// ------------ UART_BRDConfigure --------------------
// Uses Fixed Point Arithmetic to calculate the Integer and Fractional Parts of the
// Baud-Rate divisor required to work with the desired baud rate. It automatically
// chooses if High-Speed Mode should be enabled depending on the parameters provided
// https://stackoverflow.com/questions/10067510/fixed-point-arithmetic-in-c-programming
//
// Input: SYS_CLOCK - The clock frequency of the system
//        baudRate - The desired baud rate for UART transmission
// Output: None
static void UART_BRDConfigure(uint32_t SYS_CLOCK, uint32_t baudRate) {
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

// --------- UART_LCRHConfigure -------------
// Input: wordLength - The number of bits in the data word
//                     3 for 8 bits, 2 for 7 bits, 1 for 6 bits, and otherwise for 5 bits
//        parity - If the LSB is 1, it denotes parity is enabled
//                 the 2nd bit being a 1 denotes Even Parity
//        useTwoStopBits - For two stop bits to be used at the end of transmission
// Output: None
static void UART_LCRHConfigure(uint8_t wordLength, uint8_t parity, bool useTwoStopBits) {
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

// -------- UART_Enable -------
// Enables the UART, Transmit and Receive operations
// Input: None
// Output: None
void UART_Enable(void) {
  UART1_CTL_R |= UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
}

// -------- UART_Disable -------
// Waits for the UART to be IDLE before clearing the FIFO by disabling it and
// then disables the UART, Transmit and Receive operations
// Input: None
// Output: None
void UART_Disable(void) {
  // Wait for transmission to finish
  while (UART1_FR_R & UART_FR_BUSY)
    ;

  UART1_LCRH_R &= (unsigned)~UART_LCRH_FEN;                                  // Clear the FIFO
  UART1_CTL_R &= (unsigned)~(UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE); // Disable the UART
}

static void UART1_NVIC_Enable(void) {
  NVIC_EN0_R |= NVIC_EN0_INT6;                                                                               // Enable Interrupt 6 for UART 1
  NVIC_PRI1_R = (NVIC_PRI1_R & (unsigned)~NVIC_PRI1_INT6_M) | (UART_INTERRUPT_PRIORITY << NVIC_PRI1_INT6_S); // Set Priority to 5
}

// -------- UART_InterruptInit -------
// Initializes the UART's Interrupt handler for the Receive FIFO Level and Timeout events for software flow control,
// also setting the Priority to 5
// Input: None
// Output: None
void UART_TimeoutInterrupt(void) {
  // Allow Receive FIFO and Timeout interrupts on to be handled by controller
  UART1_IM_R = UART_IM_RXIM;
  UART1_NVIC_Enable();
}

// Input: RXFIFOLevel - The desired level to trigger the Receive FIFO interrupt on
void UART_FIFOInterrupt(uint8_t RXFIFOLevel) {
  UART1_IM_R |= UART_IM_RTIM;

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

  UART1_NVIC_Enable();
}

// ----------- UART_Init ------------
// Initializes the UART 1 module in Port C using the specified options
// Input: SYS_CLOCK - System Clock
//        baudRate - The desired baud rate for UART transmission
//        wordLength - The number of bits in the data word
//                     3 for 8 bits, 2 for 7 bits, 1 for 6 bits, and otherwise for 5 bits
//        parity - If the LSB is 1, it denotes parity is enabled
//                 the 2nd bit being a 1 denotes Even Parity
//        useTwoStopBits - For two stop bits to be used at the end of transmission
// Output: None
void UART_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t parity, bool useTwoStopBits) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // Enable Port B's clock
  SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1; // Enable UART module 1's clock

  GPIO_PORTB_AFSEL_R |= RX_BIT | TX_BIT; // Enable Alternate functions in PINS 4 and 5.
  GPIO_PORTB_PCTL_R =
      (GPIO_PORTB_PCTL_R & ~PCTL_MASK) | PCTL; // Enable UART Tx and Rx functions by masking the pin peripherals byte and setting the UART value
  GPIO_PORTB_DIR_R = (GPIO_PORTB_DIR_R & ~RX_BIT) | TX_BIT; // Configure Pin 4 (UART1Rx) as an input and Pin 5 (UART1Tx) as an output
  GPIO_PORTB_DEN_R |= RX_BIT | TX_BIT;                      // Enable Digital on PINS 4 and 5.
  GPIO_PORTB_AMSEL_R &= ~(RX_BIT | TX_BIT);                 // Disable Analog on PINS 4 and 5.

  UART_Disable(); // Disable UART

  UART_BRDConfigure(SYS_CLOCK, baudRate);                 // Set Baud-Rate Divisor (BRD)
  UART_LCRHConfigure(wordLength, useTwoStopBits, parity); // Configure (Line Control) LCRH

  UART_Enable(); // Enable UART
}

// ----------- UART_Transmit -------------
// Transmits data through the UART line. If the transmit FIFO is full, it blocks further
// processing until there is space to prevent data loss
// Input: data - Data buffer to transmit
//        byteCount - The number of bytes in the data buffer to transmit
// Output: None
void UART_Transmit(uint8_t *data, uint32_t byteCount) {
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

// --------- UART_Receive ------------
// Gets the data received by the UART. It waits for the Receive FIFO to not be empty,
// before returning the data
// Input: None
// Output: Data received from UART
void UART_Receive(uint8_t *data, uint32_t length) {
  uint32_t lop = 0;
  while (length > 0) {
    while (UART1_FR_R & UART_FR_RXFE) { // Wait for Receive FIFO to have data
      ++lop;
    }

    *data = (uint8_t)UART1_DR_R; // Read data

    if (--length > 0) // Increment pointer if there's still data to write
      data++;
  }
}
