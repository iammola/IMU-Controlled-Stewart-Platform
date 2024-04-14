/**
 * @file RTC.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "tm4c123gh6pm.h"

/**
 * @brief
 * @param SYS_CLOCK
 */
void RTC_TimerInit(uint32_t SYS_CLOCK);

/**
 * @brief
 * @param
 */
extern inline void RTC_Handler(void);

/**
 * @brief
 * @param
 */
static inline void RTC_TimerStop(void) {
  TIMER2_CTL_R &= (unsigned)~TIMER_CTL_TAEN; // Disable Timer 2A
}

/**
 * @brief
 * @param
 */
static inline void RTC_TimerStart(void) {
  TIMER2_CTL_R |= TIMER_CTL_TAEN; // Enable Timer 2A
}
