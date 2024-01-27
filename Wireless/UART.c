#include <stdint.h>
#include <stdbool.h>

#include "UART.h"
#include "tm4c123gh6pm.h"

#define FRACTIONAL_BRD_MULTIPLIER 6 // 2^6 = 64 or LSH 6 times
#define FRACTIONAL_BRD_MASK (1 << FRACTIONAL_BRD_MULTIPLIER) - 1

// https://stackoverflow.com/questions/10067510/fixed-point-arithmetic-in-c-programming
static void UART_BRDConfigure(uint8_t SYS_CLOCK, bool useHighSpeed, uint32_t baudRate)
{
  uint8_t CLK_DIV = useHighSpeed ? 8 : 16;
  // BRD = SYS_CLOCK / (CLK_DIV * BAUD_RATE);
  const uint32_t BRD = ((SYS_CLOCK << FRACTIONAL_BRD_MULTIPLIER) / (CLK_DIV * baudRate / 1e6));

  // Dividing it by the initial multiplier would then give us back the integer value
  uint32_t integerPart = BRD >> FRACTIONAL_BRD_MULTIPLIER;

  // This is the result after multiplying it by 2^multiplier (LSH) required to calculate the fraction part
  // to get the actual decimal value, multiply by it by 10 to the power of the desired DP and RSH by the multiplier to divide
  uint32_t fractionalPart = (BRD & FRACTIONAL_BRD_MASK) + 0.5;

  // Write calculated BRD integer part
  UART4_IBRD_R = integerPart;

  // Write calculated fractional part
  UART4_FBRD_R = fractionalPart;
}

static void UART_LCRHConfigure(uint8_t wordLength, bool useTwoStopBits, bool isEvenParity)
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
  if (isEvenParity)
    result |= UART_LCRH_EPS;

  // Enable Two Stop Bits
  if (useTwoStopBits)
    result |= UART_LCRH_STP2;

  UART4_LCRH_R = result;
}

static void UART_Enable(bool useHighSpeed)
{
  uint32_t result = UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;

  // Enable High-Speed Mode
  if (!useHighSpeed)
    result |= UART_CTL_HSE;

  UART4_CTL_R |= result;
}

static void UART_Disable(void)
{
  // Wait for transmission to finish
  while (UART4_FR_R & UART_FR_BUSY)
    ;

  // Clear the FIFO
  UART4_LCRH_R &= ~UART_LCRH_FEN;

  // Disable the UART
  UART4_CTL_R &= ~UART_CTL_UARTEN;
}

static void UART_InterruptEnable(uint8_t RXFIFOLevel)
{
  // Allow Receive interrupts on to be handled by controller
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

// TODO: Support dynamically choosing port
void UART_Init(uint8_t SYS_CLOCK, bool useHighSpeed, uint32_t baudRate, uint8_t wordLength, uint8_t RXFIFOLevel, bool useTwoStopBits, bool isEvenParity)
{
  // Enable Port C's clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOC;

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
  UART_BRDConfigure(SYS_CLOCK, useHighSpeed, baudRate);

  // Configure (Line Control) LCRH
  UART_LCRHConfigure(wordLength, useTwoStopBits, isEvenParity);

  // Enable UART 4 Interrupts
  UART_InterruptEnable(RXFIFOLevel);

  // Enable UART
  UART_Enable(useHighSpeed);
}

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

void UART_Receive(uint8_t byteCount, uint8_t *dest)
{
  uint8_t byteIndex = 0;

  do
  {
    // Wait FOR Receive FIFO to have data
    while (UART4_FR_R & UART_FR_RXFE)
      ;

    // Read data
    dest[byteIndex] = UART4_DR_R;

    // Increment index tracker
    byteIndex++;
  } while (byteIndex < byteCount);
}
