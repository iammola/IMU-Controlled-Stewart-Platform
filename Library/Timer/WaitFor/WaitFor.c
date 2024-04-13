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
 * @param LOAD
 * @return
 */
bool WaitFor(uint32_t LOAD) {
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // Enable Timer Module 1

  while ((SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R1) == 0x00) { // Wait for Module ready
  }

  if (TIMER1_CTL_R & TIMER_CTL_TAEN) // Wait For Timer already in use
    return false;

  TIMER1_CTL_R &= (unsigned)~TIMER_CTL_TAEN; // Disable Timer A

  TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
  TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT; // Periodic timer
  TIMER1_TAILR_R = LOAD;                  // Set LOAD

  TIMER1_CTL_R |= TIMER_CTL_TAEN;

  return true;
}
