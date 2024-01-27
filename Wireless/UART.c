#include <stdint.h>
#include <stdbool.h>

#include "UART.h"
#include "tm4c123gh6pm.h"

#define FRACTIONAL_BRD_MULTIPLIER 6 // 2^6 = 64 or LSH 6 times
#define FRACTIONAL_BRD_MASK (1 << FRACTIONAL_BRD_MULTIPLIER) - 1

static void UART_BRDConfigure(uint32_t SYS_CLOCK, uint32_t baudRate);
static void UART_LCRHConfigure(uint8_t wordLength, bool useTwoStopBits, bool useEvenParity);
static void UART_Enable(void);
static void UART_Disable(void);
static void UART_InterruptEnable(uint8_t RXFIFOLevel);

// ------------ UART_BRDConfigure --------------------
// Uses Fixed Point Arithmetic to calculate the Integer and Fractional Parts of the
// Baud-Rate divisor required to work with the desired baud rate. It automatically
// chooses if High-Speed Mode should be enabled depending on the parameters provided
// https://stackoverflow.com/questions/10067510/fixed-point-arithmetic-in-c-programming
//
// Input: SYS_CLOCK - The clock frequency of the system
//        baudRate - The desired baud rate for UART transmission
// Output: None
static void UART_BRDConfigure(uint32_t SYS_CLOCK, uint32_t baudRate)
{
  // High speed mode must be used if the BAUD rate would run faster than the system clock
  bool needsHighSpeed = (baudRate * 16) > SYS_CLOCK;

  // Basically the Baud-Rate Generation formula given, BRD = SYS_CLOCK / (CLK_DIV * BAUD_RATE);
  // But uses Fixed Point Arithmetic by multiplying the values by specific power of 2
  // Depending on the need for the High Speed Mode, the CLK_DIV changes
  uint32_t BRD = (SYS_CLOCK << FRACTIONAL_BRD_MULTIPLIER) / (baudRate * (needsHighSpeed ? 8 : 16));

  // Dividing it by the initial multiplier would then give us back the integer value
  uint32_t integerPart = BRD >> FRACTIONAL_BRD_MULTIPLIER;

  // The fractional part would be stored in the number of LSBs shifted by the multiplier. Therefore the
  // multiplication would not be required again. All that'll be left to do will be to filter that part out
  // using a mask with all the bits it was shifted by as ones (1). i.e (2 ^ multiplier) - 1.
  // The value would be a number between 0 and 2^multiplier. Where 0 would be 0.0, half of 2^M would be 0.5, and vice versa.
  uint32_t fractionalPart = (BRD & FRACTIONAL_BRD_MASK) + 0.5;

  // Write calculated BRD integer part
  UART4_IBRD_R = integerPart;

  // Write calculated fractional part
  UART4_FBRD_R = fractionalPart;

  // Enable High-Speed Mode
  if (needsHighSpeed)
    UART4_CTL_R |= UART_CTL_HSE;
  else
    // Disable High-Speed Mode
    UART4_CTL_R &= ~UART_CTL_HSE;
}

// --------- UART_LCRHConfigure -------------
// Input: wordLength - The number of bits in the data word
//                     3 for 8 bits, 2 for 7 bits, 1 for 6 bits, and otherwise for 5 bits
//        useTwoStopBits - For two stop bits to be used at the end of transmission
//        useEvenParity - Specifies if even or odd parity should be used
// Output: None
static void UART_LCRHConfigure(uint8_t wordLength, bool useTwoStopBits, bool useEvenParity)
{
  // Enable FIFO buffers and Parity
  uint32_t result = UART_LCRH_FEN | UART_LCRH_PEN;

  // Specify word length, falling back to 5 bits.
  switch (wordLength)
  {
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

  // Enable Even Parity
  if (useEvenParity)
    result |= UART_LCRH_EPS;

  // Enable Two Stop Bits
  if (useTwoStopBits)
    result |= UART_LCRH_STP2;

  UART4_LCRH_R = result;
}

// -------- UART_Enable -------
// Enables the UART, Transmit and Receive operations
// Input: None
// Output: None
static void UART_Enable(void)
{
  UART4_CTL_R |= UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
}

// -------- UART_Disable -------
// Waits for the UART to be IDLE before clearing the FIFO by disabling it and
// then disables the UART, Transmit and Receive operations
// Input: None
// Output: None
static void UART_Disable(void)
{
  // Wait for transmission to finish
  while (UART4_FR_R & UART_FR_BUSY)
    ;

  // Clear the FIFO
  UART4_LCRH_R &= ~UART_LCRH_FEN;

  // Disable the UART
  UART4_CTL_R &= ~(UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
}

// -------- UART_Disable -------
// Enables the UART's Interrupt handler for the Receive FIFO Level and Timeout events for software flow control,
// also setting the Priority to 5
// Input: RXFIFOLevel - The desired level to trigger the Receive FIFO interrupt on
// Output: None
static void UART_InterruptEnable(uint8_t RXFIFOLevel)
{
  // Allow Receive FIFO and Timeout interrupts on to be handled by controller
  UART4_IM_R = UART_IM_RXIM | UART_IM_RTIM;

  // Set RX Interrupt Levels
  switch (RXFIFOLevel)
  {
  case 4:
    RXFIFOLevel = UART_IFLS_RX7_8;
    break;
  case 3:
    RXFIFOLevel = UART_IFLS_RX6_8;
    break;
  case 1:
    RXFIFOLevel = UART_IFLS_RX2_8;
    break;
  case 0:
    RXFIFOLevel = UART_IFLS_RX1_8;
    break;
  default:
    RXFIFOLevel = UART_IFLS_RX4_8;
    break;
  }

  // Set RX FIFO level
  UART4_IFLS_R = (UART4_IFLS_R & ~UART_IFLS_RX_M) | RXFIFOLevel;

  // Enable Interrupt 60 (would be 60 - 32 = 28 on EN1) for UART 4
  NVIC_EN1_R |= NVIC_EN0_INT28;

  // Set Priority to 5
  NVIC_PRI15_R = (NVIC_PRI15_R & ~NVIC_PRI15_INTA_M) | (UART_INTERRUPT_PRIORITY << NVIC_PRI15_INTA_S);
}

// ----------- UART_Init ------------
// Initializes the UART 4 module in Port C using the specified options
// Input: SYS_CLOCK - System Clock
//        baudRate - The desired baud rate for UART transmission
//        wordLength - The number of bits in the data word
//                     3 for 8 bits, 2 for 7 bits, 1 for 6 bits, and otherwise for 5 bits
//        RXFIFOLevel - The desired level to trigger the Receive FIFO interrupt on
//        useTwoStopBits - For two stop bits to be used at the end of transmission
//        useEvenParity - Specifies if even or odd parity should be used
// Output: None
void UART_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t RXFIFOLevel, bool useTwoStopBits, bool useEvenParity)
{
  // Enable Port C's clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;

  // Enable UART module 0's clock
  SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R4;

  // Enable Alternate functions in PINS 4 and 5.
  GPIO_PORTC_AFSEL_R |= (1 << 4) | (1 << 5);

  // Enable UART Tx and Rx functions by masking the pin peripherals byte and setting the UART value
  GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & ~(GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M)) | GPIO_PCTL_PC4_U4RX | GPIO_PCTL_PC5_U4TX;

  // Configure Pin 4 (UART4Rx) as an input and Pin 5 (UART4Tx) as an output
  GPIO_PORTC_DIR_R = (GPIO_PORTC_DIR_R & ~0x30) | (1 << 5);

  // Enable Digital on PINS 4 and 5.
  GPIO_PORTC_DEN_R |= (1 << 4) | (1 << 5);

  // Disable UART
  UART_Disable();

  // Set Baud-Rate Divisor (BRD)
  UART_BRDConfigure(SYS_CLOCK, baudRate);

  // Configure (Line Control) LCRH
  UART_LCRHConfigure(wordLength, useTwoStopBits, useEvenParity);

  // Enable UART 4 Interrupts
  UART_InterruptEnable(RXFIFOLevel);

  // Enable UART
  UART_Enable();
}

// ----------- UART_Transmit -------------
// Transmits data through the UART line. If the transmit FIFO is full, it blocks further
// processing until there is space to prevent data loss
// Input: data - Data buffer to transmit
//        byteCount - The number of bytes in the data buffer to transmit
// Output: None
void UART_Transmit(uint8_t *data, uint8_t byteCount)
{
  uint8_t byteIndex = 0;

  do
  {
    // Prevent Data from being set while the transmit FIFO is full
    while (UART4_FR_R & UART_FR_TXFF)
      ;

    // Transmit data byte
    UART4_DR_R = data[byteIndex];

    // Increment index tracker
    byteIndex++;
  } while (byteIndex < byteCount);
}

// --------- UART_Receive ------------
// Gets the data received by the UART. It waits for the Receive FIFO to not be empty,
// before returning the data
// Input: None
// Output: Data received from UART
uint8_t UART_Receive(void)
{
  // Wait FOR Receive FIFO to have data
  while (UART4_FR_R & UART_FR_RXFE)
    ;

  // Read data
  return UART4_DR_R;
}
