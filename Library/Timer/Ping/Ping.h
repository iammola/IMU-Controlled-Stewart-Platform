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

#include <stdbool.h>
#include <stdint.h>

#include "tm4c123gh6pm.h"

/**
 * @brief
 * @param LOAD
 * @param oneShot
 */
void Ping_TimerInit(uint32_t LOAD, bool oneShot);

/**
 * @brief
 * @param
 */
extern inline void Ping_Handler(void);

/**
 * @brief
 * @param
 */
static inline void Ping_TimerReset(void) {
  TIMER0_TAV_R = 0; // Reset value
}

/**
 * @brief 
 * @param  
 */
static inline void Ping_TimerDisable(void) {
  TIMER0_CTL_R &= (unsigned)~TIMER_CTL_TAEN; // Disable Timer A
}

/**
 * @brief 
 * @param  
 */
static inline void Ping_TimerEnable(void) {
  TIMER0_CTL_R |= TIMER_CTL_TAEN; // Enable Timer A
}
