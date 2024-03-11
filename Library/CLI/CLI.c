/**
 * Name: Ademola Adedeji
 * Date: 27th January, 2024
 * Course-Code: ELTR-2400
 * Course-Name: Communication Systems II
 *
 * Lab Description: Initializes UART module 0 on Port A pins 4 (Tx) and 5 (Tx) for connection with the
 *                  COM port of the PC. IT Automatically calculates the need for High Speed Mode (HSE),
 *                  and the Integer and Fractional parts of the Baud Rate. It allows for configuration
 *                  of the Data length, Use of Parity and number of stop bits. Only uses Receive interrupts,
 *                  and also allows for configuration of the FIFO level to trigger the Receive FIFO interrupt.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "CLI.h"
#include "tm4c123gh6pm.h"

#define RX_BIT 1 << 0
#define TX_BIT 1 << 1
#define PINS   (unsigned)(RX_BIT | TX_BIT)

#define PCTL      (unsigned)(GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX)
#define PCTL_MASK (unsigned)(GPIO_PCTL_PA0_M | GPIO_PCTL_PA1_M)

#define FRACTIONAL_BRD_MULTIPLIER 6 // 2^6 = 64 or LSH 6 times
#define FRACTIONAL_BRD_MASK       (1 << FRACTIONAL_BRD_MULTIPLIER) - 1

/* Calculates and sets the Integer and Fractional of the BRD and HSE */
static void UART_BRDConfigure(uint32_t SYS_CLOCK, uint32_t baudRate);
/* Sets the LCRH configuration */
static void UART_LCRHConfigure(uint8_t wordLength, uint8_t parity, bool useTwoStopBits);
/* Enables the UART module, TX and RX operations */
static void UART_Enable(void);
/* Disables the UART module */
static void UART_Disable(void);
/* Enables the Receive Timeout and FIFO interrupts */
static void UART_InterruptInit(uint8_t RXFIFOLevel);

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

  // Write calculated BRD integer part
  UART0_IBRD_R = integerPart;

  // Write calculated fractional part
  UART0_FBRD_R = fractionalPart;

  // Enable High-Speed Mode
  if (needsHighSpeed)
    UART0_CTL_R |= UART_CTL_HSE;
  else
    // Disable High-Speed Mode
    UART0_CTL_R &= (unsigned)~UART_CTL_HSE;
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
    case 3:
      result |= UART_LCRH_WLEN_8;
      break;
    case 2:
      result |= UART_LCRH_WLEN_7;
      break;
    case 1:
      result |= UART_LCRH_WLEN_6;
      break;
    default:
      result |= UART_LCRH_WLEN_5;
      break;
  }

  // If LSB is 1, then Parity should be enabled
  if (parity & 0x1) {
    // Enable Parity
    result |= UART_LCRH_PEN;

    // Enable Even Parity
    if (parity & 0x2)
      result |= UART_LCRH_EPS;
  }

  // Enable Two Stop Bits
  if (useTwoStopBits)
    result |= UART_LCRH_STP2;

  UART0_LCRH_R = result;
}

// -------- UART_Enable -------
// Enables the UART, Transmit and Receive operations
// Input: None
// Output: None
static void UART_Enable(void) {
  UART0_CTL_R |= UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
}

// -------- UART_Disable -------
// Waits for the UART to be IDLE before clearing the FIFO by disabling it and
// then disables the UART, Transmit and Receive operations
// Input: None
// Output: None
static void UART_Disable(void) {
  // Wait for transmission to finish
  while (UART0_FR_R & UART_FR_BUSY)
    ;

  // Clear the FIFO
  UART0_LCRH_R &= (unsigned)~UART_LCRH_FEN;

  // Disable the UART
  UART0_CTL_R &= (unsigned)~(UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
}

// -------- UART_InterruptEnable -------
// Enables the UART's Interrupt handler for the Receive FIFO Level and Timeout events for software flow control,
// also setting the Priority to 5
// Input: RXFIFOLevel - The desired level to trigger the Receive FIFO interrupt on
// Output: None
static void UART_InterruptEnable(uint8_t RXFIFOLevel) {
  // Set RX Interrupt Levels
  switch (RXFIFOLevel) {
    case RX_FIFO_7_8:
      RXFIFOLevel = UART_IFLS_RX7_8;
      break;
    case RX_FIFO_6_8:
      RXFIFOLevel = UART_IFLS_RX6_8;
      break;
    case RX_FIFO_4_8:
      RXFIFOLevel = UART_IFLS_RX4_8;
      break;
    case RX_FIFO_2_8:
      RXFIFOLevel = UART_IFLS_RX2_8;
      break;
    case RX_FIFO_1_8:
      RXFIFOLevel = UART_IFLS_RX1_8;
      break;
    default:
      // Do not enable interrupt if not given an explicit value
      return;
  }

  // Allow Receive FIFO and Timeout interrupts on to be handled by controller
  UART0_IM_R = UART_IM_RXIM | UART_IM_RTIM;

  // Set RX FIFO level
  UART0_IFLS_R = (UART0_IFLS_R & (unsigned)~UART_IFLS_RX_M) | RXFIFOLevel;

  NVIC_EN0_R |= NVIC_EN0_INT5;                                                                               // Enable Interrupt 5 for UART0
  NVIC_PRI1_R = (NVIC_PRI1_R & (unsigned)~NVIC_PRI1_INT5_M) | (UART_INTERRUPT_PRIORITY << NVIC_PRI1_INT5_S); // Set Priority
}

// ----------- CLI_Init ------------
// Initializes the UART 4 module in Port C using the specified options
// Input: SYS_CLOCK - System Clock
//        baudRate - The desired baud rate for UART transmission
//        wordLength - The number of bits in the data word
//                     3 for 8 bits, 2 for 7 bits, 1 for 6 bits, and otherwise for 5 bits
//        RXFIFOLevel - The desired level to trigger the Receive FIFO interrupt on
//        parity - If the LSB is 1, it denotes parity is enabled
//                 the 2nd bit being a 1 denotes Even Parity
//        useTwoStopBits - For two stop bits to be used at the end of transmission
// Output: None
void CLI_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t RXFIFOLevel, uint8_t parity, bool useTwoStopBits) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; // Enable Port A's clock
  SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // Enable UART module 0's clock

  GPIO_PORTA_AFSEL_R |= PINS;                                  // Enable Alternate functions in PINS.
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~PCTL_MASK) | PCTL; // Configure Peripherals
  GPIO_PORTA_DIR_R = (GPIO_PORTA_DIR_R & ~PINS) | TX_BIT;      // Configure I/O pins
  GPIO_PORTA_DEN_R |= PINS;                                    // Enable Digital on PINS

  UART_Disable(); // Disable UART

  UART_BRDConfigure(SYS_CLOCK, baudRate);                 // Set Baud-Rate Divisor (BRD)
  UART_LCRHConfigure(wordLength, useTwoStopBits, parity); // Configure (Line Control) LCRH
  UART_InterruptEnable(RXFIFOLevel);                      // Enable UART 4 Interrupts

  UART_Enable(); // Enable UART
}

// ----------- CLI_Write -------------
// Transmits data through the UART line. If the transmit FIFO is full, it blocks further
// processing until there is space to prevent data loss
// Input: data - String to transmit
// Output: None
void CLI_Write(char *data) {
  uint8_t  byteIndex = 0;
  uint32_t length = strlen((const char *)data);

  do {
    // Prevent Data from being set while the transmit FIFO is full
    while (UART0_FR_R & UART_FR_TXFF)
      ;

    // Transmit data byte
    UART0_DR_R = data[byteIndex];

    // Increment index tracker
    byteIndex++;
  } while (byteIndex < length);
}

// --------- CLI_Read ------------
// Gets the data received by the UART. It waits for the Receive FIFO to not be empty,
// before returning the data
// Input: None
// Output: Data received from UART
uint8_t CLI_Read(void) {
  // Wait FOR Receive FIFO to have data
  while (UART0_FR_R & UART_FR_RXFE)
    ;

  // Read data
  return (uint8_t)UART0_DR_R;
}
