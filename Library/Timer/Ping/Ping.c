/**
 * @file Ping.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "tm4c123gh6pm.h"

#include "Ping.h"

void TIMER0A_Handler(void) {
  TIMER0_ICR_R |= TIMER_ICR_TATOCINT; // Clear interrupt
  Ping_Handler();
}

/**
 * @brief
 * @param SYS_CLOCK
 */
void Ping_TimerInit(uint32_t SYS_CLOCK) {
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0; // Enable Timer Module 0

  TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR; // Periodic timer and counting up
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;                           // Trigger interrupt when reaches limit
  TIMER0_TAILR_R = SYS_CLOCK / PING_FREQUENCY;                // Set LOAD

  NVIC_EN0_R |= NVIC_EN0_INT19;                                                  // Enable Timer 0 SubTimer A Interrupt Handler
  NVIC_PRI14_R = (NVIC_PRI14_R & ~NVIC_PRI4_INT19_M) | (7 << NVIC_PRI4_INT19_S); // Set Priority
}

void Ping_TimerReset(void) {
  TIMER0_TAV_R = 0x00; // Reset value
}
