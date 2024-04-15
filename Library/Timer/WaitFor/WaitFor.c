/**
 * @file WaitFor.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "WaitFor.h"

/**
 * @brief
 * @param
 */
void WaitFor_TimerInit(void) {
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // Enable Timer Module 1

  while ((SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R1) == 0x00) { // Wait for Module ready
  }

  TIMER1_CTL_R &= (unsigned)~TIMER_CTL_TAEN; // Disable Timer A

  TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
  TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT; // 1 shot timer

  /* Enable and Disable Timer to get Timeout interrupt flag set */
  TIMER1_TAILR_R = 10;
  TIMER1_CTL_R |= TIMER_CTL_TAEN | TIMER_CTL_TASTALL; // enable Timer 2A
  while (!WaitFor_TimerIsDone())                      // Wait for finish
    ;
  TIMER1_CTL_R &= (unsigned)~TIMER_CTL_TAEN; // Disable Timer 2A
}

/**
 * @brief
 * @param LOAD
 */
void WaitFor_TimerStart(uint32_t LOAD) {
  TIMER1_TAILR_R = LOAD;                              // Set LOAD
  TIMER1_ICR_R |= TIMER_ICR_TATOCINT;                 // clear interrupt
  TIMER1_CTL_R |= TIMER_CTL_TAEN | TIMER_CTL_TASTALL; // enable timer
}
