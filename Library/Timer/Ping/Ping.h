/**
 * @file Ping.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */
 
 #include <stdint.h>

#define PING_INTERRUPT_PRIORITY (unsigned)4

void Ping_TimerInit(uint32_t LOAD);

void Ping_Handler(void);

void Ping_TimerReset(void) ;
