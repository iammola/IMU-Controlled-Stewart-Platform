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

/**
 * @brief
 * @param LOAD
 */
void Ping_TimerInit(uint32_t LOAD);

/**
 * @brief
 * @param
 */
extern inline void Ping_Handler(void);

/**
 * @brief
 * @param
 */
void Ping_TimerReset(void);
