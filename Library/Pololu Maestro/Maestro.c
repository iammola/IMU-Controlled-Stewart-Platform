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

#define SET_TARGET_CMD    0x84
#define MOVING_STATE_CMD  0x93
#define GET_POSITIONS_CMD 0x90
#define GO_HOME_CMD       0xA2

#define SERVO_IS_MOVING 0x01

#define FULL_ROTATION    90.0f
#define ZERO_PULSE_POS   1500.0f
#define FULL_PULSE_WIDTH (2400.0f - ZERO_PULSE_POS)

static void Maestro_WaitForIdle(void);

void Maestro_Init(uint32_t SYS_CLOCK) {
  UART1_Init(SYS_CLOCK, Maestro_BAUD, WORD_8_BIT, NO_PARITY, ONE_STOP_BIT);

  GPIO_PORTB_ODR_R |= UART1_TX_BIT;  // Enable Open-Drain for Pull-Up to 5V
  GPIO_PORTB_DR4R_R |= UART1_TX_BIT; // Use 4mA drive, for intended 5V
}

void Maestro_SetAngles(float angles[Maestro_Channels]) {
  float angle = 0.0f;

  uint8_t  channel = 0;
  uint32_t pulseWidth = 0;
  uint8_t  cmd[4] = {SET_TARGET_CMD, 0, 0, 0};

  for (channel = 0; channel < Maestro_Channels; channel++) {
    angle = angles[channel];

    // Keep in bounds
    if (angle > FULL_ROTATION)
      angle = FULL_ROTATION;
    else if (angle < -FULL_ROTATION)
      angle = -FULL_ROTATION;

    // Get pulse width from zero position, in us converted to quarter-us
    pulseWidth = (uint32_t)((ZERO_PULSE_POS + ((angle * FULL_PULSE_WIDTH) / FULL_ROTATION)) * 4);

    cmd[1] = channel;                    // Set channel
    cmd[2] = pulseWidth & 0xFF;          // lower byte of quarter-us
    cmd[3] = (pulseWidth & 0xFF00) >> 8; // upper byte of quarter-us

    UART1_Transmit(cmd, 4); // send data

    if (channel > 0)
      Maestro_WaitForIdle(); // Wait for servo to finish moving
  }
}

static void Maestro_WaitForIdle(void) {
  uint8_t isMovingState = SERVO_IS_MOVING;
  uint8_t isMoving = MOVING_STATE_CMD;

  while (isMovingState == SERVO_IS_MOVING) {
    UART1_Transmit(&isMoving, 1);
    UART1_Receive(&isMovingState, 1);
  }
}

void Maestro_GetPositions(void) {
  float angles[Maestro_Channels] = {0.0f};

  uint8_t  channel = 0;
  uint32_t pulseWidth = 0;

  uint8_t cmd[2] = {GET_POSITIONS_CMD, 0};
  uint8_t response[2] = {0.0f};

  for (channel = 0; channel < Maestro_Channels; channel++) {
    cmd[1] = channel;       // Set channel
    UART1_Transmit(cmd, 2); // send data

    UART1_Receive(response, 2); // read response

    // Get pulse width in quarter-us, convert back to us
    pulseWidth = (uint32_t)((response[1] << 8) | response[0]) / 4;

    // reversed formula to calculate angle from returned pulse width
    angles[channel] = (((float)pulseWidth - ZERO_PULSE_POS) * FULL_ROTATION) / FULL_PULSE_WIDTH;
  }
}

void Maestro_GoHome(void) {
  uint8_t cmd = GO_HOME_CMD;
  UART1_Transmit(&cmd, 1);
}
