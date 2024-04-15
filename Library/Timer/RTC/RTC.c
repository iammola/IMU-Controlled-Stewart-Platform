/**
 * @file RTC.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "RTC.h"

#define RTC_INTERRUPT_PRIORITY (unsigned)4

void TIMER2A_Handler(void);
void TIMER2A_Handler(void) {
  TIMER2_ICR_R |= TIMER_ICR_TATOCINT; // Clear interrupt
  RTC_Handler();
}

/**
 * @brief
 * @param SYS_CLOCK
 */
void RTC_TimerInit(uint32_t SYS_CLOCK) {
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2; // Enable Timer Module 2

  while ((SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R2) == 0x00) { // Wait for Module ready
  }

  TIMER2_CTL_R &= (unsigned)~TIMER_CTL_TAEN; // Disable Timer Module 2A

  TIMER2_TAMR_R &= (unsigned)~TIMER_TAMR_TAMR_M; // Clear Timer mode
  TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;         // RTC config
  TIMER2_TAILR_R = SYS_CLOCK / 1000;             // Set the match value to ticks required for 1ms

  TIMER2_ICR_R |= TIMER_ICR_TATOCINT; // Clear interrupt
  TIMER2_IMR_R |= TIMER_IMR_TATOIM;   // Allow interrupt trigger

  NVIC_EN0_R |= NVIC_EN0_INT23;                                                                     // Enable Timer 2 SubTimer A Interrupt Handler
  NVIC_PRI5_R = (NVIC_PRI5_R & ~NVIC_PRI5_INT23_M) | (RTC_INTERRUPT_PRIORITY << NVIC_PRI5_INT23_S); // Set Priority
}
