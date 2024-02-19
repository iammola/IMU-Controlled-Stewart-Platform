#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "SysTick.h"
#include "tm4c123gh6pm.h"
#include "UART/UART.h"

#include "HC-12.h"

#define CMD_MODE_BIT    (unsigned)(1 << 2) // PB2
#define CMD_MODE_PCTL_M (unsigned)(GPIO_PCTL_PB2_M)
#define CMD_MODE_ADDR   (*((volatile uint32_t *)(0x40005000 | (CMD_MODE_BIT << 2))))

#define VCC_BIT    (unsigned)(1 << 3) // PB3
#define VCC_PCTL_M (unsigned)(GPIO_PCTL_PB3_M)
#define VCC_ADDR   (*((volatile uint32_t *)(0x40005000 | (VCC_BIT << 2))))

static void HC12_Config(uint32_t SYS_CLOCK, uint32_t BAUD);
static bool HC12_SendCommand(char *FORMAT, char *RESPONSE_FORMAT, ...);

bool    HasNewData = false;
uint8_t RX_Data_Buffer[MAX_MESSAGE_SIZE] = {0};

static uint8_t SYN = 0;

void UART1_Handler(void);

void UART1_Handler(void) {
  uint8_t dataLength = 0;

  UART1_ICR_R |= UART_ICR_RXIC | UART_ICR_RTIC;

  UART_Receive(&SYN, 1);
  if (SYN != SYNC_WORD)
    return;

  UART_Receive(&dataLength, 1);
  if (dataLength > MAX_MESSAGE_SIZE)
    dataLength = MAX_MESSAGE_SIZE;

  UART_Receive(RX_Data_Buffer, dataLength);
  HasNewData = true;
}

static bool HC12_SendCommand(char *FORMAT, char *RESPONSE_FORMAT, ...) {
  char cmd[25];
  char response[25];
  char CMDResponse[25] = {0};

  va_list argptr;
  va_start(argptr, RESPONSE_FORMAT);               // Last fixed parameter
  snprintf(cmd, 25, FORMAT, argptr);               // Format CMD string
  snprintf(response, 25, RESPONSE_FORMAT, argptr); // Format expected response
  va_end(argptr);

  UART_Transmit((uint8_t *)cmd, strlen((const char *)cmd));
  UART_Receive((uint8_t *)CMDResponse, strlen((const char *)response));

  return strcmp(CMDResponse, response) == 0;
}

static void HC12_CommandMode(void) {
  VCC_ADDR = 0; // Drive VCC pin LOW to turn off device

  CMD_MODE_ADDR = 0;   // Drive SET pin LOW for AT Command Mode
  SysTick_Wait10ms(5); // Wait for 40ms to enter AT Command Mode

  VCC_ADDR = VCC_BIT; // Drive VCC pin HIGH to turn device on with CMD mode
}

static void HC12_TransmissionMode(void) {
  VCC_ADDR = 0; // Drive VCC pin LOW to turn off device

  CMD_MODE_ADDR = CMD_MODE_BIT; // Drive SET pin HIGH for Transmission mode
  SysTick_Wait10ms(21);         // Wait for 200ms (Page 5) [80ms (Page 8)] to enter Transmission Mode

  VCC_ADDR = VCC_BIT; // Drive VCC pin HIGH to turn device on with Transmission mode
}

static void HC12_Config(uint32_t SYS_CLOCK, uint32_t BAUD) {
  UART_Init(SYS_CLOCK, 9600, WORD_8_BIT, NO_PARITY, ONE_STOP_BIT); // Starts at Default config 9600 bps, 8-N-1 control

  HC12_CommandMode(); // Enter Command Mode

  HC12_SendCommand("AT+V", "www.hc01.com HC-12 v2.6");     // Verify Firmware version matches
  HC12_SendCommand("AT+B%d", "OK+B%d", BAUD);              // Set Baud-Rate to desired bps
  HC12_SendCommand("AT+U%d%c%d", "OK+U%d%c%d", 8, 'N', 1); // Set to 8 data-bits, No Parity, 1 stop-bit
  HC12_SendCommand("AT+P%d", "OK+P%d", 8);                 // Set transmitting power to 20dBm

  UART_Disable();
}

void HC12_Init(uint32_t SYS_CLOCK, uint32_t BAUD, RX_Data_Handler OnRX) {
  SysTick_Init();

  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // Enable Port B

  GPIO_PORTB_DIR_R |= (CMD_MODE_BIT | VCC_BIT);      // Use as Output pin
  GPIO_PORTB_DEN_R |= (CMD_MODE_BIT | VCC_BIT);      // Enable Digital Mode
  GPIO_PORTB_AMSEL_R &= ~(CMD_MODE_BIT | VCC_BIT);   // Disable Analog Mode
  GPIO_PORTB_AFSEL_R &= ~(CMD_MODE_BIT | VCC_BIT);   // Disable Alternate functions
  GPIO_PORTB_PCTL_R &= ~(CMD_MODE_PCTL_M | VCC_BIT); // Clear Peripheral functions
  GPIO_PORTB_DR4R_R |= VCC_BIT;                      // Use 4mA drive for VCC pin

  HC12_Config(SYS_CLOCK, BAUD);

  UART_Init(SYS_CLOCK, BAUD, WORD_8_BIT, NO_PARITY, ONE_STOP_BIT); // Use new Baud-Rate
  UART_TimeoutInterrupt();
  UART_FIFOInterrupt(RX_FIFO_6_8); // Use 3/4 full for 12 bytes out of 16

  HC12_TransmissionMode();
}

bool HC12_SendData(uint8_t *data, uint8_t length) {
  uint8_t TX_Metadata[METADATA_SIZE] = {SYNC_WORD, length};

  if (length > MAX_MESSAGE_SIZE)
    return false;

  UART_Transmit(TX_Metadata, METADATA_SIZE);
  UART_Transmit(data, length);

  return true;
}
