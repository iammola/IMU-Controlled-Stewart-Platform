#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "tm4c123gh6pm.h"
#include "UART/UART.h"

#include "HC-12.h"

#define CMD_MODE_BIT    (unsigned)(1 << 2) // PB2
#define CMD_MODE_PCTL_M (unsigned)(GPIO_PCTL_PB2_M)
#define CMD_MODE_ADDR   (*((volatile uint32_t *)(0x40005000 | (CMD_MODE_BIT << 2))))

static void HC12_Config(uint32_t SYS_CLOCK, uint32_t BAUD);
static bool HC12_SendCommand(char *FORMAT, char *RESPONSE_FORMAT, ...);

bool    HasNewData = false;
uint8_t RX_Data_Buffer[7] = {0};

static uint8_t LAST_SENT_ACK = 0;

void UART1_Handler(void) {
  UART1_ICR_R |= UART_ICR_RTIC | UART_ICR_RXIC;
}

static bool HC12_SendCommand(char *FORMAT, char *RESPONSE_FORMAT, ...) {
  char cmd[15];
  char response[15];
  char CMDResponse[10] = {0};

  va_list argptr;
  va_start(argptr, RESPONSE_FORMAT);               // Last fixed parameter
  snprintf(cmd, 15, FORMAT, argptr);               // Format CMD string
  snprintf(response, 15, RESPONSE_FORMAT, argptr); // Format expected response
  va_end(argptr);

  CMD_MODE_ADDR = 0; // Drive SET pin LOW for AT Command Mode

  UART_Transmit((uint8_t *)cmd, strlen((const char *)cmd));
  UART_Receive((uint8_t *)CMDResponse, strlen((const char *)response));

  CMD_MODE_ADDR = CMD_MODE_BIT; // Drive SET pin HIGH for normal mode

  return strcmp(CMDResponse, response);
}

static void HC12_Config(uint32_t SYS_CLOCK, uint32_t BAUD) {
  UART_Init(SYS_CLOCK, 9600, WORD_8_BIT, NO_PARITY, ONE_STOP_BIT); // Starts at Default config 9600 bps, 8-N-1 control

  HC12_SendCommand("AT+V", "HC-12_V1.1");                  // Verify Firmware version matches
  HC12_SendCommand("AT+B%d", "OK+B%d", BAUD);              // Set Baud-Rate to 115200 bps
  HC12_SendCommand("AT+U%d%c%d", "OK+U%d%c%d", 8, 'N', 1); // Set to 8 data-bits, No Parity, 1 stop-bit
  HC12_SendCommand("AT+P%d", "OK+P%d", 8);                 // Set transmitting power to 20dBm

  UART_Disable();
}

void HC12_Init(uint32_t SYS_CLOCK, uint32_t BAUD, RX_Data_Handler OnRX) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // Enable Port B

  GPIO_PORTB_DIR_R |= CMD_MODE_BIT;      // Use as Output pin
  GPIO_PORTB_DEN_R |= CMD_MODE_BIT;      // Enable Digital Mode
  GPIO_PORTB_AMSEL_R &= ~CMD_MODE_BIT;   // Disable Analog Mode
  GPIO_PORTB_AFSEL_R &= ~CMD_MODE_BIT;   // Disable Alternate functions
  GPIO_PORTB_PCTL_R &= ~CMD_MODE_PCTL_M; // Clear Peripheral functions

  HC12_Config(SYS_CLOCK, BAUD);

  UART_Init(SYS_CLOCK, BAUD, WORD_8_BIT, NO_PARITY, ONE_STOP_BIT); // Use new Baud-Rate
  UART_TimeoutInterrupt();
  UART_FIFOInterrupt(RX_FIFO_4_8); // Use half-full for 8 bytes out of 16
}

bool HC12_SendData(uint8_t *data, uint8_t length) {
  uint8_t payloadIdx = 0;
  uint8_t TX_Payload[8] = {0};

  if (length > 7)
    return false;

  TX_Payload[payloadIdx++] = ++LAST_SENT_ACK;

  for (; payloadIdx < length + 1; payloadIdx++) {
    TX_Payload[payloadIdx] = data[payloadIdx - 1];
  }

  return true;
}
