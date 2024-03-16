/**
 * @file UART1.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <stdbool.h>
#include <stdint.h>

#include "UART.h"

void UART1_Enable(void);

void UART1_Disable(void);

void UART1_TimeoutInterrupt(uint8_t interruptPriority);

void UART1_FIFOInterrupt(uint8_t RXFIFOLevel, uint8_t interruptPriority);

void UART1_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t parity, bool useTwoStopBits);

void UART1_Transmit(uint8_t *data, uint32_t length);

bool UART1_Receive(uint8_t *data, uint32_t length);
