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

#define PING_FREQUENCY 2

void Ping_TimerInit(uint32_t SYS_CLOCK);

void Ping_Handler(void);

void Ping_TimerReset(void) ;
