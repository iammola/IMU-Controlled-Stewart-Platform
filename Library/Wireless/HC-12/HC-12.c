/**
 * @file HC-12.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief Simple library to communicate with the HC-12 433 MHz wireless module
 * @version 0.1
 * @date 2024-03-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "PORT_BASE.h"
#include "SysTick/SysTick.h"
#include "tm4c123gh6pm.h"
#include "UART/UART5.h"

#include "HC-12.h"

#define SET_BIT    (unsigned)(1 << 1) // PE1
#define SET_PCTL_M (unsigned)(GPIO_PCTL_PE1_M)
#define SET_ADDR   (*((volatile uint32_t *)(PORTE_BASE | (SET_BIT << 2))))

#define VCC_BIT    (unsigned)(1 << 2) // PE2
#define VCC_PCTL_M (unsigned)(GPIO_PCTL_PE2_M)
#define VCC_ADDR   (*((volatile uint32_t *)(PORTE_BASE | (VCC_BIT << 2))))

typedef enum { TRANSMISSION_MODE = 0x4B, COMMAND_MODE = 0xEA } MODE;

static void HC12_SetMode(MODE NewMode);
static bool HC12_SendCommand(char *FORMAT, char *RESPONSE_FORMAT, ...);

static MODE   CURRENT_MODE = 0xFF;
volatile bool HasNewData = false;

uint8_t RX_Data_Buffer[MAX_MESSAGE_SIZE] = {0};
uint8_t TX_Data_Buffer[MAX_MESSAGE_SIZE] = {0};

static const uint8_t INTERRUPT_PRIORITY = 5;

void UART5_Handler(void);

/**
 * @brief UART5 Interrupt Handler
 * @param
 */
void UART5_Handler(void) {
  uint8_t SYN = 0;
  uint8_t dataLength = 0;
  bool    success = false;

  UART5_ICR_R |= UART_ICR_RXIC | UART_ICR_RTIC;

  success = UART5_Receive(&SYN, 1);
  if (!success || SYN != SYNC_WORD) // Ensure sync word matches to assume valid data
    return;

  success = UART5_Receive(RX_Data_Buffer, 1); // Get amount of bytes sent
  if (!success)
    return;

  if (RX_Data_Buffer[0] > MAX_MESSAGE_SIZE) // Ensure it's within expected range
    RX_Data_Buffer[0] = MAX_MESSAGE_SIZE;

  success = UART5_Receive(RX_Data_Buffer + 1, dataLength); // Read data
  if (!success)
    return;

  HasNewData = true; // Toggle to alert for new data
}

/**
 * @brief Variadic utility function to construct the command string for sending
 * commands to the HC-12 module.
 * @param FORMAT Command string with placeholders for values
 * @param RESPONSE_FORMAT Expected response string with placeholders for values
 * @param ... Values to be used as placeholders
 * @return `true` if the expected response was read. `false` otherwise
 */
static bool HC12_SendCommand(char *FORMAT, char *RESPONSE_FORMAT, ...) {
  char cmd[25];
  char response[25];
  char CMDResponse[25] = {0};

  va_list argptrCMD, argptrRES;
  va_start(argptrCMD, RESPONSE_FORMAT); // Last fixed parameter
  va_copy(argptrRES, argptrCMD);        // copy arguments for response format

  vsnprintf(cmd, 25, FORMAT, argptrCMD); // Format CMD string
  va_end(argptrCMD);
  vsnprintf(response, 25, RESPONSE_FORMAT, argptrRES); // Format expected response
  va_end(argptrRES);

  if (CURRENT_MODE != COMMAND_MODE) // Confirm in Command Mode
    HC12_SetMode(COMMAND_MODE);

  UART5_Transmit((uint8_t *)cmd, strlen((const char *)cmd));
  UART5_Receive((uint8_t *)CMDResponse, strlen((const char *)response));

  return strcmp(CMDResponse, response) == 0; // Verify they are equal
}

/**
 * @brief Used to toggle the SET pin of the device, to transition between Command
 * and Transmission mode
 * @param NewMode `true` to drive the pin High for Transmission Mode, `false`
 * for Command Mode
 */
static void HC12_SetMode(MODE NewMode) {
  if (CURRENT_MODE == NewMode)
    return; // Ignore repeated disables of the device

  VCC_ADDR = VCC_BIT; // Drive VCC pin HIGH to turn off device

  SET_ADDR = (NewMode == TRANSMISSION_MODE) ? SET_BIT : 0x00; // Drive SET pin HIGH for mode
  SysTick_Wait10ms(35);                                       // Wait for 200ms to enter Transmission Mode or 40ms to enter AT command mode

  VCC_ADDR = 0; // Drive VCC pin LOW to turn device on with mode
  CURRENT_MODE = NewMode;
}

/**
 * @brief Initializes the HC-12 VCC and SET pins. Required to call `HC12_Config` to setup
 * the UART pins at the desired Baud Rate
 * @param
 */
void HC12_Init(void) {
  SysTick_Init();

  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;                 // Enable Port E
  while ((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R4) == 0x00) { // Wait for Port ready
  }

  GPIO_PORTE_DIR_R |= (SET_BIT | VCC_BIT);         // Use as Output pin
  GPIO_PORTE_DEN_R |= (SET_BIT | VCC_BIT);         // Enable Digital Mode
  GPIO_PORTE_AMSEL_R &= ~(SET_BIT | VCC_BIT);      // Disable Analog Mode
  GPIO_PORTE_AFSEL_R &= ~(SET_BIT | VCC_BIT);      // Disable Alternate functions
  GPIO_PORTE_PCTL_R &= ~(SET_PCTL_M | VCC_PCTL_M); // Clear Peripheral functions
  GPIO_PORTE_DR8R_R |= VCC_BIT;                    // Use 8mA drive for VCC pin

  HC12_SetMode(TRANSMISSION_MODE);
}

/**
 * @brief Configures the HC-12 module to use the desired Baud Rate and TX Power Level.
 * Verifies the Firmware version matches the expectation, using Channel 1 (433 MHz) and
 * FU3 transmission mode
 * @param SYS_CLOCK System clock speed
 * @param baud Desired baud rate for communication, changes the Over-the-Air baud rate
 * @param powerLevel Desired power level for transmission
 */
void HC12_Config(uint32_t SYS_CLOCK, BAUD_RATE baud, TX_POWER powerLevel) {
  HC12_SetMode(COMMAND_MODE); // Enter Command Mode

  UART5_Init(SYS_CLOCK, 9600, WORD_8_BIT, NO_PARITY, ONE_STOP_BIT); // Starts at Default config 9600 bps, 8-N-1 control

  while (!HC12_SendCommand("AT", "OK")) { // Poll until Command is accepted
  }

  HC12_SendCommand("AT+V", "www.hc01.com HC-12 v2.6"); // Verify Firmware version matches
  HC12_SendCommand("AT+B%d", "OK+B%d", baud);          // Set Baud-Rate to desired bps
  HC12_SendCommand("AT+P%d", "OK+P%d", powerLevel);    // Set transmitting power desired level
  HC12_SendCommand("AT+C%03d", "OK+C%03d", 1);         // Use channel 1
  HC12_SendCommand("AT+FU%d", "OK+FU%d", 3);           // Set Transmission Mode to FU3

  UART5_Disable();

  HC12_SetMode(TRANSMISSION_MODE); // Exit Command Mode

  UART5_Init(SYS_CLOCK, baud, WORD_8_BIT, NO_PARITY, ONE_STOP_BIT); // Use new Baud-Rate
  UART5_TimeoutInterrupt(INTERRUPT_PRIORITY);
  UART5_FIFOInterrupt(RX_FIFO_1_8, INTERRUPT_PRIORITY); // Use 1/8 full for 2 bytes (metadata) out of 16
}

/**
 * @brief Used to send data at the configured baud rate through the air
 * @param data
 * @param length
 * @return
 */
bool HC12_SendData(uint8_t *data, uint8_t length) {
  uint8_t TX_Metadata[METADATA_SIZE] = {SYNC_WORD, length};

  if (length > MAX_MESSAGE_SIZE)
    return false;

  if (CURRENT_MODE != TRANSMISSION_MODE) // Confirm in transmission mode
    HC12_SetMode(TRANSMISSION_MODE);

  UART5_Transmit(TX_Metadata, METADATA_SIZE);
  UART5_Transmit(data, length);

  return true;
}
