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
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------

//      <o> INPUT: Joystick Source
//              < 0=>  0: PS2 Joystick (ADC)
//              < 1=>  1: PS5 Controller (UART - Web)
//          <i> Source of Joystick data
//
#define JOYSTICK_SOURCE 1

//-------- <<< end of configuration section >>> ------------------------------

#include <math.h>

#include "tm4c123gh6pm.h"

#include "Joystick.h"

static volatile Position *__position;

static inline void UpdateData(float x1, float y1, float x2, float y2) {
  float  angle = atan2f(y2, x2);                                                          // Use 2nd Joystick for tilt
  Coords translation = {.x = x1 * PAN_RANGE, .y = y1 * PAN_RANGE, .z = 0.0f * PAN_RANGE}; // Use 1st joystick for pan

  __position->quaternion = normalizeQuaternion(-13.0f, -cosf(angle), sinf(angle), 0);
  __position->translation = translation;
  __position->isNew = true;
}

#if JOYSTICK_SOURCE == 0
#define TIMER_ENABLE()  TIMER0_CTL_R |= TIMER_CTL_TAEN | TIMER_CTL_TAOTE; // Enable Timer A and trigger ADC
#define TIMER_DISABLE() TIMER0_CTL_R &= (unsigned)~TIMER_CTL_TAEN;        // Disable Timer A
#define ADC_ENABLE()    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;                  // Enable Sample Sequencer 1
#define ADC_DISABLE()   ADC0_ACTSS_R &= (unsigned)~ADC_ACTSS_ASEN1;       // Disable Sample Sequencer 1

void ADC0SS1_Handler(void);

static void Joystick_ADC_Init(uint32_t SYS_CLOCK, uint16_t SAMPLING_FREQ);
static void Joystick_Timer_Init(uint32_t LOAD);

void ADC0SS1_Handler(void) {
  UpdateData(                            // Calculate co-ordinates, from -X, -Y (-1) to X, Y (1)
      (ADC0_SSFIFO1_R / 2048.0f) - 1.0f, // 1st sample
      (ADC0_SSFIFO1_R / 2048.0f) - 1.0f, // 2nd sample
      (ADC0_SSFIFO1_R / 2048.0f) - 1.0f, // 3rd sample
      (ADC0_SSFIFO1_R / 2048.0f) - 1.0f  // 4th sample
  );

  ADC0_ISC_R |= ADC_ISC_IN1; // Clear SS1 Interrupt
}

/**
 * @brief
 * @param SYS_CLOCK
 * @param SAMPLING_FREQ
 * @param position
 */
void Joystick_Init(uint32_t SYS_CLOCK, uint16_t SAMPLING_FREQ, volatile Position *position) {
  __position = position;

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

  ADC0_SSMUX1_R = (V2Ry_AIN << ADC_SSMUX1_MUX3_S) | // Sample V2Ry in 4th sample
                  (V2Rx_AIN << ADC_SSMUX1_MUX2_S) | // Sample V2Rx in 3rd sample
                  (V1Ry_AIN << ADC_SSMUX1_MUX1_S) | // Sample V1Ry in 2nd sample
                  (V1Rx_AIN << ADC_SSMUX1_MUX0_S);  // Sample V1Rx in 1st sample
  ADC0_SSCTL1_R = ADC_SSCTL1_END3 | ADC_SSCTL1_IE3; // Use 4th sample for end of sequence and trigger interrupt

  ADC0_ISC_R |= ADC_ISC_IN1;    // Clear Sample Sequencer 1 interrupts
  ADC0_IM_R |= ADC_IM_MASK1;    // Unmask Sample Sequencer 1 interrupts
  NVIC_EN0_R |= NVIC_EN0_INT15; // Enable ADC module 0, sequence 1's Interrupt Handler
  NVIC_PRI3_R = (NVIC_PRI3_R & (unsigned)~NVIC_PRI3_INT15_M) | (JOYSTICK_INT_PRIORITY << NVIC_PRI3_INT15_S); // Set Interrupt priority
}
#elif JOYSTICK_SOURCE == 1

#include <stdio.h>
#include <string.h>

#include "CLI/CLI.h"

/**
 * @brief
 * @param SYS_CLOCK
 * @param SAMPLING_FREQ
 * @param position
 */
void Joystick_Init(uint32_t SYS_CLOCK, uint16_t SAMPLING_FREQ, volatile Position *position) {
  __position = position;

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT);
  CLI_Disable();
}

void Joystick_Enable(void) {
  // Allow Receive FIFO and Timeout interrupts on to be handled by controller
  UART0_IM_R = UART_IM_RXIM | UART_IM_RTIM;

  // Set RX FIFO level
  UART0_IFLS_R = (UART0_IFLS_R & (unsigned)~UART_IFLS_RX_M) | UART_IFLS_RX2_8;

  NVIC_EN0_R |= NVIC_EN0_INT5;                                                         // Enable Interrupt 5 for UART0
  NVIC_PRI1_R = (NVIC_PRI1_R & (unsigned)~NVIC_PRI1_INT5_M) | (6 << NVIC_PRI1_INT5_S); // Set Priority

  CLI_Enable(); // Enable UART Module
}

void Joystick_Disable(void) {
  CLI_Disable(); // Disable UART
}

static uint8_t data[16] = {0};
static float   converted[4] = {0};

void UART0_Handler(void) {
  uint8_t idx = 0;
  uint8_t match = 0xEA;

  UART0_ICR_R |= UART_ICR_RTIC | UART_ICR_RXIC; // Clear interrupt

  do {
    match = CLI_Read();
  } while ((match != 0xEA) && (UART0_FR_R&UART_FR_RXFE));

  if (match != 0xEA)
    return;

  for (idx = 0; idx < 16; idx++) {
    data[idx] = CLI_Read();
  }

  memcpy(converted, &data, 16);
  UpdateData(converted[0], converted[1], converted[2], converted[3]);
}

#endif
