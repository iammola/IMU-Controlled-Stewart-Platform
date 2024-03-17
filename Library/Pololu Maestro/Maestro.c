/**
 * @file Maestro.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "tm4c123gh6pm.h"

#include "Maestro.h"
#include "UART/UART1.h"

#define Maestro_BAUD 225e3

#define SET_TARGET_CMD   0x84
#define FULL_ROTATION    90.0f
#define ZERO_PULSE_POS   1500.0f
#define FULL_PULSE_WIDTH (2400.0f - ZERO_PULSE_POS)

void Maestro_Init(uint32_t SYS_CLOCK) {
  UART1_Init(SYS_CLOCK, Maestro_BAUD, WORD_8_BIT, NO_PARITY, ONE_STOP_BIT);

  GPIO_PORTB_ODR_R |= UART1_TX_BIT;  // Enable Open-Drain for Pull-Up to 5V
  GPIO_PORTB_DR4R_R |= UART1_TX_BIT; // Use 4mA drive, for intended 5V
}

void Maestro_SetAngles(float angles[Maestro_Channels]) {
  float    angle = 0.0f;
  uint32_t channel;
  uint32_t pulseWidth = 0;
  uint8_t  bytes[4] = {SET_TARGET_CMD, 0, 0, 0};

  for (channel = 0; channel < Maestro_Channels; channel++) {
    angle = angles[channel];

    // Keep in bounds
    if (angle > FULL_ROTATION)
      angle = FULL_ROTATION;
    else if (angle < -FULL_ROTATION)
      angle = -FULL_ROTATION;

    // Get pulse width from zero position, in us converted to quarter-us
    pulseWidth = (uint32_t)((ZERO_PULSE_POS + ((angle * FULL_PULSE_WIDTH) / FULL_ROTATION)) * 4);

    bytes[1] = channel;
    bytes[2] = pulseWidth & 0xFF;
    bytes[3] = (pulseWidth & 0xFF00) >> 8;

    UART1_Transmit(bytes, 4);
  }
}
