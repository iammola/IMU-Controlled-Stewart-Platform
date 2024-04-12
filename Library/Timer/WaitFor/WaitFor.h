/**
 * @file WaitFor.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-12
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <stdbool.h>

#include "tm4c123gh6pm.h"

/**
 * @brief 
 * @param LOAD 
 * @return 
 */
bool WaitFor(uint32_t LOAD);

static inline bool WaitFor_IsDone(void) {
  return TIMER1_RIS_R & TIMER_RIS_TATORIS;
}

static inline void WaitFor_Stop(void) {
  TIMER1_CTL_R &= (unsigned)~TIMER_CTL_TAEN; // Disable Timer 1A
}
