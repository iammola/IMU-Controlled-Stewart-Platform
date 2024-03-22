/**
 * @file Joystick.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <math.h>

#include "tm4c123gh6pm.h"

#include "Joystick.h"

#define TIMER_ENABLE()  TIMER0_CTL_R |= TIMER_CTL_TAEN | TIMER_CTL_TAOTE; // Enable Timer A and trigger ADC
#define TIMER_DISABLE() TIMER0_CTL_R |= (unsigned)~TIMER_CTL_TAEN;        // Disable Timer A
#define ADC_ENABLE()    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;                  // Enable Sample Sequencer 1
#define ADC_DISABLE()   ADC0_ACTSS_R |= (unsigned)~ADC_ACTSS_ASEN1;       // Disable Sample Sequencer 1

void ADC0SS1_Handler(void);

static void Joystick_ADC_Init(uint32_t SYS_CLOCK, uint16_t SAMPLING_FREQ);
static void Joystick_Timer_Init(uint32_t LOAD);

static volatile bool       *__hasNewData;
static volatile Quaternion *__quatDest;

void ADC0SS1_Handler(void) {
  // Calculate co-ordinates, from -X, -Y (-1) to X, Y (1)
  float x = (ADC0_SSFIFO1_R / 2048.0f) - 1.0f; // 1st sample
  float y = (ADC0_SSFIFO1_R / 2048.0f) - 1.0f; // 2nd sample
  
  float angle = atan2f(-y, -x);

  // Clear SS1 Interrupt
  ADC0_ISC_R |= ADC_ISC_IN1;

  *__quatDest = normalizeQuaternion(-13.0f, -cosf(angle), sinf(angle), 0);
  *__hasNewData = true;
}

/**
 * @brief
 * @param SYS_CLOCK
 * @param SAMPLING_FREQ
 * @param quatDest
 * @param hasNewData
 */
void Joystick_Init(uint32_t SYS_CLOCK, uint16_t SAMPLING_FREQ, volatile Quaternion *quatDest, volatile bool *hasNewData) {
  __quatDest = quatDest;
  __hasNewData = hasNewData;

  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;                 // Enable Port D
  while ((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R3) == 0x00) { // Wait for Port ready
  }

  GPIO_PORTD_DIR_R &= ~PINS;                                // Configure as inputs
  GPIO_PORTD_DEN_R &= ~PINS;                                // Set SW pin as digital pin
  GPIO_PORTD_AMSEL_R |= PINS;                               // Set VRx, VRy as analog pins
  GPIO_PORTD_AFSEL_R |= PINS;                               // select alternate functions on desired pins
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & ~PCTL_M) | PCTL; // choose peripheral functions

  Joystick_ADC_Init(SYS_CLOCK, SAMPLING_FREQ);
}

/**
 * @brief
 * @param
 */
void Joystick_Enable(void) {
  TIMER_ENABLE()
  ADC_ENABLE()
}

/**
 * @brief
 * @param
 */
void Joystick_Disable(void) {
  TIMER_DISABLE()
  ADC_DISABLE()
}

/**
 * @brief
 * @param LOAD
 */
static void Joystick_Timer_Init(uint32_t LOAD) {
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;                   // Enable Timer Module 0
  while ((SYSCTL_RCGCTIMER_R & SYSCTL_RCGCTIMER_R0) == 0x00) { // Wait for Port ready
  }

  TIMER_DISABLE()

  TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // Periodic timer
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;       // Trigger interrupt when reaches limit
  TIMER0_TAILR_R = LOAD;                  // Set Load value
}

/**
 * @brief
 * @param SYS_CLOCK
 * @param SAMPLING_FREQ
 */
static void Joystick_ADC_Init(uint32_t SYS_CLOCK, uint16_t SAMPLING_FREQ) {
  SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;                 // Enable ADC module 0 clock
  while ((SYSCTL_PRADC_R & SYSCTL_RCGCADC_R0) == 0x00) { // Wait for Module ready
  }

  Joystick_Timer_Init(SYS_CLOCK / SAMPLING_FREQ);

  ADC_DISABLE()

  ADC0_EMUX_R = (ADC0_EMUX_R & (unsigned)~ADC_EMUX_EM1_M) | ADC_EMUX_EM1_TIMER; // Select Timer for Sample Sequencer 1

  ADC0_SSMUX1_R = (ADC0_SSMUX1_R & (unsigned)~(ADC_SSMUX1_MUX1_M | ADC_SSMUX1_MUX0_M)) | (VRy_AIN << ADC_SSMUX1_MUX1_S) | // Sample VRy in 2nd sample
                  (VRx_AIN << ADC_SSMUX1_MUX0_S);                                                                         // Sample VRx in 1st sample
  ADC0_SSCTL1_R = ADC_SSCTL1_END1 | ADC_SSCTL1_IE1; // Use 2nd sample for end of sequence and trigger interrupt

  ADC0_ISC_R |= ADC_ISC_IN1;    // Clear Sample Sequencer 1 interrupts
  ADC0_IM_R |= ADC_IM_MASK1;    // Unmask Sample Sequencer 1 interrupts
  NVIC_EN0_R |= NVIC_EN0_INT15; // Enable ADC module 0, sequence 1's Interrupt Handler
  NVIC_PRI3_R = (NVIC_PRI3_R & (unsigned)~NVIC_PRI3_INT15_M) | (JOYSTICK_INT_PRIORITY << NVIC_PRI3_INT15_S); // Set Interrupt priority
}
