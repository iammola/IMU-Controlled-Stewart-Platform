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
#include <stdio.h>
#include <string.h>

#include "tm4c123gh6pm.h"

#include "CLI/CLI.h"
#include "Joystick.h"

#define JOYSTICK_INT_PRIORITY 2

#define PAN_RANGE 30.0f

static volatile Position *__position;

void UART0_Handler(void) {
  float   angle = 0.0f;
  float   axes[4] = {0};
  uint8_t data[16] = {0};

  uint8_t idx = 0;
  uint8_t match = (uint8_t)UART0_DR_R; // Get sync word

  UART0_ICR_R = UART_ICR_RXIC; // Clear interrupt

  if (match != 0xEA) {
    while (!(UART0_FR_R & UART_FR_RXFE)) {
      match = (uint8_t)UART0_DR_R; // Clear all the remaining invalid data
    }

    return;
  }

  for (idx = 0; idx < 16; idx++)
    data[idx] = CLI_Read();

  if (!__position->inUse) {
    memcpy(axes, &data, 16); // Create 4 32-bit floats from 16 8-bit numbers
    angle = atan2f(-axes[3], -axes[2]);

    __position->quaternion = normalizeQuaternion(-13.0f, -cosf(angle), sinf(angle), 0);                            // Use 2nd Joystick for tilt
    __position->translation = (Coords){.x = axes[1] * PAN_RANGE, .y = axes[0] * PAN_RANGE, .z = 0.0f * PAN_RANGE}; // Use 1st joystick for pan
  }
}

/**
 * @brief
 * @param SYS_CLOCK
 * @param position
 */
void Joystick_Init(uint32_t SYS_CLOCK, volatile Position *position) {
  __position = position;

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT);
  CLI_Disable();
}

void Joystick_Enable(void) {
  // Allow Receive FIFO and Timeout interrupts on to be handled by controller
  UART0_IM_R = UART_IM_RXIM;

  // Set RX FIFO level
  UART0_IFLS_R = (UART0_IFLS_R & (unsigned)~UART_IFLS_RX_M) | UART_IFLS_RX1_8;

  NVIC_EN0_R |= NVIC_EN0_INT5;                                                                             // Enable Interrupt 5 for UART0
  NVIC_PRI1_R = (NVIC_PRI1_R & (unsigned)~NVIC_PRI1_INT5_M) | (JOYSTICK_INT_PRIORITY << NVIC_PRI1_INT5_S); // Set Priority

  CLI_Enable(); // Enable UART Module
}

void Joystick_Disable(void) {
  CLI_Disable(); // Disable UART
}
