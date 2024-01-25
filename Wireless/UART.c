#include <stdint.h>
#include <stdbool.h>

#include "UART.h"
#include "tm4c123gh6pm.h"

#define DP 4
#define INTERRUPT_PRIORITY 5
#define FRACTIONAL_BRD_MULTIPLIER 6 // 2^6= 6 or LSH 4 times
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
  UART0_IBRD_R = integerPart;

  // Write calculated fractional part
  UART0_FBRD_R = fractionalPart;
}

static void UART_LCRHConfigure(uint8_t wordLength, bool useTwoStopBits, bool isEvenParity, bool enableFIFO)
{
  uint32_t result = 0;

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

  // Enable FIFO buffers
  if (enableFIFO)
    result |= UART_LCRH_FEN;

  // Enable Parity
  result |= UART_LCRH_PEN;

  // Enable Even Parity
  if (isEvenParity)
    result |= UART_LCRH_EPS;

  // Enable Two Stop Bits
  if (useTwoStopBits)
    result |= UART_LCRH_STP2;

  UART0_LCRH_R = result;
}

static void UART_Enable(bool useHighSpeed)
{
  uint8_t result = UART_CTL_UARTEN;

  // Enable High-Speed Mode
  if (!useHighSpeed)
    result |= UART_CTL_HSE;

  UART0_CTL_R |= result;
}

static void UART_Disable(void)
{
  // Wait for transmission to finish
  while (UART0_FR_R & UART_FR_BUSY)
    ;

  // Clear the FIFO
  UART0_LCRH_R &= ~UART_LCRH_FEN;

  // Disable the UART
  UART0_CTL_R &= ~UART_CTL_UARTEN;
}

static void UART_InterruptEnable(void)
{
  // Clear RX interrupts
  UART0_ICR_R = UART_IM_RXIM;

  // Allow Receive interrupts on to be handled by controller
  UART0_IM_R = UART_IM_RXIM;

  // Enable Interrupt 5 for UART 0
  NVIC_EN0_R |= NVIC_EN0_INT5;

  // Set Priority to 5
  NVIC_PRI1_R = (NVIC_PRI1_R & ~(NVIC_PRI1_INT5_M)) | (INTERRUPT_PRIORITY << NVIC_PRI1_INT5_S);
}

static void UART0_Handler(void) {

}

// TODO: Support dynamically choosing port
void UART_Init(uint8_t SYS_CLOCK, bool useHighSpeed, uint32_t baudRate, uint8_t wordLength, bool useTwoStopBits, bool isEvenParity, bool enableFIFO)
{
  // Enable Port A's clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOA;

  // Enable UART module 0's clock
  SYSCTL_RCGCUART_R |= SYSCTL_RCGC1_UART0;

  // Enable Alternate functions in PINS 0 and 1.
  GPIO_PORTA_AFSEL_R |= (1 << 0) | (1 << 1);

  // Enable UART Tx and Rx functions by masking the pin peripherals byte and setting the UART value
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~(GPIO_PCTL_PA0_M | GPIO_PCTL_PA1_M)) | GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX;

  // Configure Pin 0 (UART0Rx) as an input and Pin 1 (UART0Tx) as an output
  GPIO_PORTA_DIR_R = (GPIO_PORTA_DIR_R & ~0x3) | (1 << 1);

  // Enable Digital on PINS 0 and 1.
  GPIO_PORTA_DEN_R |= 0x3;

  // Disable UART
  UART_Disable();

  // Set Baud-Rate Divisor (BRD)
  UART_BRDConfigure(SYS_CLOCK, useHighSpeed, baudRate);

  // Configure (Line Control) LCRH
  UART_LCRHConfigure(wordLength, useTwoStopBits, isEvenParity, enableFIFO);

  // Enable UART 0 Interrupts
  UART_InterruptEnable();

  // Enable UART
  UART_Enable(useHighSpeed);
}

void UART_Transmit(void) {}

void UART_Receive(void) {}
