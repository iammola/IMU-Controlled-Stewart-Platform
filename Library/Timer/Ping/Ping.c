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

#include "Ping.h"

#define PING_INTERRUPT_PRIORITY (unsigned)0

void TIMER0A_Handler(void);

void TIMER0A_Handler(void) {
  TIMER0_ICR_R |= TIMER_ICR_TATOCINT; // Clear interrupt
  Ping_Handler();
}

/**
 * @brief
 * @param LOAD
 * @param oneShot
 */
void Ping_TimerInit(uint32_t LOAD, bool oneShot) {
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0; // Enable Timer Module 0

  Ping_TimerDisable();

  TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;
  TIMER0_TAMR_R = oneShot ? TIMER_TAMR_TAMR_1_SHOT : TIMER_TAMR_TAMR_PERIOD; // Periodic timer and counting up
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;                                          // Trigger interrupt when reaches limit
  TIMER0_TAILR_R = LOAD;                                                     // Set LOAD

  NVIC_EN0_R |= NVIC_EN0_INT19;                                                                      // Enable Timer 0 SubTimer A Interrupt Handler
  NVIC_PRI4_R = (NVIC_PRI4_R & ~NVIC_PRI4_INT19_M) | (PING_INTERRUPT_PRIORITY << NVIC_PRI4_INT19_S); // Set Priority
}
