#ifndef WIRED
#include <stdio.h>

#include "tm4c123gh6pm.h"

#include "UART.h"

#define NAME_INSTRUCTION "NAME"
#define BAUD_INSTRUCTION "BAUD"
#define PASSWORD_INSTRUCTION "PSS"
#define PASSWORD_REQUIRED_INSTRUCTION "ISCEN"

#define MAX_DATA_SIZE 18

#define PASSWORD "Kloppo#6"
#define BAUD_RATE "0"         // 115200 bps
#define PASSWORD_REQUIRED "1" // Require Passwords

// Take away 1 for the trailing NULL character
#define STR_LEN(arr) (sizeof(arr) / sizeof(arr[0])) - 1

#define PWRC_BIT 6 // PC6
#define PWRC_ADDR (*((volatile uint32_t *)(0x40006000 | (1 << (PWRC_BIT + 2)))))

#ifdef IS_DEVICE_1
#define DEVICE_NAME "A"
#define PEER_DEVICE_ADDRESS "C5"
#else
#define DEVICE_NAME "MOLA-JDY"
#define PEER_DEVICE_ADDRESS "27"
#endif

uint8_t rxIdx = 0;
uint8_t rxBuf[MAX_DATA_SIZE + 1];
uint32_t ctx = 0;
void UART4_Handler(void)
{
  while (!(UART4_FR_R & UART_FR_RXFE))
  {
    rxBuf[rxIdx] = UART_Receive();
    rxIdx++;
    if (rxIdx > MAX_DATA_SIZE)
      rxIdx = 0;
  }

  UART4_ICR_R |= UART_ICR_RXIC | UART_ICR_RTIC;
}

static void JDY10M_SendCommand(char *instruction, uint8_t instructionLen, char *value, uint8_t valueLen)
{
  uint16_t size = 5 + instructionLen + valueLen;
  char AT_Instruction[MAX_DATA_SIZE + 1];

  snprintf(AT_Instruction, size, "AT-%s%s\r\n", instruction, value);

  UART_Transmit((unsigned char *)AT_Instruction, size);
}

const uint32_t baudRates[] = {
    115200,
    57600,
    38400,
    19200,
    9600,
    4800};

void JDY10M_Init(uint32_t SYS_CLOCK)
{
  uint32_t time = 0;
  uint8_t baudIdx = 0;
  // Enable Port C's clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;

  // Disable Alternate functions on PWRC_BIT.
  GPIO_PORTC_AFSEL_R &= ~(1 << PWRC_BIT);

  // Disable Peripheral Functions on PWRC_BIT
  GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;

  // Configure PWRC pin as an output
  GPIO_PORTC_DIR_R |= 1 << PWRC_BIT;

  // Enable Digital on PWRC_BIT
  GPIO_PORTC_DEN_R |= 1 << PWRC_BIT;

  // Initialize UART for SYS_CLOCK, 115200 baud, 8 bit data length, one-eighth FIFO RX interrupts,
  // 1 stop bit, and no parity
  // https://www.electro-tech-online.com/threads/jdy-10-bluetooth-ble-uart-transceiver-module.152098/post-1312849

  // Drive PWRC bit low to send AT instructions
  PWRC_ADDR = 0;

  for (; baudIdx < 6; baudIdx++)
  {
    UART_Init(SYS_CLOCK, baudRates[baudIdx], 3 /* UART_LCRH_WLEN_8 */, 4 /* UART_IFLS_RX7_8 */, 0 /* No Parity */, false);
    // UART_Transmit((unsigned char *)"masadregui76oi8ifm0v0e9[sdgtrhsaerfkm3498rrgfre0[fewvimeraf9'fkrugfyywiero87fd6AGve0r9\r\n", 88);
    UART_Transmit((unsigned char *)"AT-BAUD\r\n", 9);
    for (time = 0; time < SYS_CLOCK; time++)
    {
    }
  }

  ctx = 0;

  // JDY10M_SendCommand(NAME_INSTRUCTION, STR_LEN(NAME_INSTRUCTION), DEVICE_NAME, STR_LEN(DEVICE_NAME));

  // JDY10M_SendCommand(BAUD_INSTRUCTION, BAUD_RATE);
  // rxBuf[1] = UART_Receive();
  // JDY10M_SendCommand(PASSWORD_INSTRUCTION, PASSWORD);
  // rxBuf[2] = UART_Receive();
  // JDY10M_SendCommand(PASSWORD_REQUIRED_INSTRUCTION, PASSWORD_REQUIRED);
  // rxBuf[3] = UART_Receive();

  // Drive PWRC bit high for transparent transmission
  PWRC_ADDR = 1 << PWRC_BIT;
}
#endif
