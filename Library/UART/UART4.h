/**
 * @file UART4.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <stdbool.h>
#include <stdint.h>

#include "UART.h"

#define UART4_RX_BIT (unsigned)(1 << 4) // PC4 - U4RX
#define UART4_TX_BIT (unsigned)(1 << 5) // PC5 - U4TX

void UART4_Enable(void);

void UART4_Disable(void);

void UART4_TimeoutInterrupt(uint8_t interruptPriority);

void UART4_FIFOInterrupt(uint8_t RXFIFOLevel, uint8_t interruptPriority);

void UART4_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t parity, bool useTwoStopBits);

void UART4_Transmit(uint8_t *data, uint32_t length);

void UART4_Receive(uint8_t *data, uint32_t length, uint32_t maxTicks);
