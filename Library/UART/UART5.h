/**
 * @file UART5.h
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

void UART5_Enable(void);

void UART5_Disable(void);

void UART5_TimeoutInterrupt(uint8_t interruptPriority);

void UART5_FIFOInterrupt(uint8_t RXFIFOLevel, uint8_t interruptPriority);

void UART5_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t parity, bool useTwoStopBits);

void UART5_Transmit(uint8_t *data, uint32_t length);

bool UART5_Receive(uint8_t *data, uint32_t length);
