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

#include "UART/UART4.h"

#include "Maestro.h"

/**
 * @brief
 * @param SYS_CLOCK
 */
void Maestro_Init(uint32_t SYS_CLOCK) {
  UART4_Init(SYS_CLOCK, Maestro_BAUD, WORD_8_BIT, NO_PARITY, ONE_STOP_BIT);

  GPIO_PORTC_ODR_R |= UART4_TX_BIT;  // Enable Open-Drain for Pull-Up to 5V
  GPIO_PORTC_DR4R_R |= UART4_TX_BIT; // Use 4mA drive, for intended 5V

  Maestro_GetErrors(); // Clear all errors
}

/**
 * @brief
 * @param channel
 * @param angle
 */
void Maestro_SetAngle(uint8_t channel, float angle) {
  uint16_t pulseWidth = 0;
  uint8_t  cmd[4] = {0};

  // Keep in bounds
  if (angle > FULL_ANGLE)
    angle = FULL_ANGLE;
  else if (angle < -FULL_ANGLE)
    angle = -FULL_ANGLE;

#if PROTOCOL == PROTOCOL__COMPACT
  // Get pulse width from zero position, in us converted to quarter-us
  pulseWidth = (NEUTRAL_PULSE_POS + (angle * K)) * 4;

  cmd[0] = CMD__SET_TARGET;
  cmd[1] = channel;                    // Set channel
  cmd[2] = pulseWidth & 0x7F;          // lower 7 bits of quarter-us
  cmd[3] = (pulseWidth & 0x7F00) >> 7; // upper 7 bits of quarter-us

  UART4_Transmit(cmd, 4); // send data
#elif PROTOCOL == PROTOCOL__MINI_SSC
  angle += FULL_ANGLE;
  pulseWidth = (uint8_t)(254.0f * (angle / (FULL_ANGLE * 2)));

  cmd[0] = CMD__SET_TARGET_MINI_SSC;
  cmd[1] = channel + MINI_SSC__OFFSET; // Set channel
  cmd[2] = (uint8_t)pulseWidth;        // Set target from 0 to 254

  UART4_Transmit(cmd, 3); // send data
#endif
}

/**
 * @brief
 * @param angles
 */
void Maestro_SetAngles(float angles[Maestro_Channels]) {
  uint8_t channel = 0;

  for (channel = 0; channel < Maestro_Channels; channel++) {
    Maestro_SetAngle(channel, angles[channel]);
  }
}

/**
 * @brief
 * @param channel
 * @return
 */
float Maestro_GetPosition(uint8_t channel) {
  float pulseWidth = 0;

  uint8_t cmd[2] = {CMD__GET_POSITIONS, channel};
  uint8_t response[2] = {0.0f};

  UART4_Transmit(cmd, 2);     // send data
  UART4_Receive(response, 2); // read response

  // Get pulse width in quarter-us, convert back to us
  pulseWidth = (float)((response[1] << 8) | response[0]) / 4.0f;

  // reversed formula to calculate angle from returned pulse width
  return (pulseWidth - NEUTRAL_PULSE_POS) / K;
}

/**
 * @brief
 * @param
 */
void Maestro_GetPositions(void) {
  uint8_t channel = 0;
  float   angles[Maestro_Channels] = {0.0f};

  for (channel = 0; channel < Maestro_Channels; channel++) {
    // reversed formula to calculate angle from returned pulse width
    angles[channel] = Maestro_GetPosition(channel);
  }
}

/**
 * @brief
 * @param
 */
void Maestro_WaitForIdle(void) {
  uint8_t scriptRunningState = (uint8_t)~SCRIPT_STATE__STOPPED;
  uint8_t cmd[] = {CMD__RESTART_SUBROUTINE_SCRIPT, 0x00};

  cmd[0] = CMD__RESTART_SUBROUTINE_SCRIPT;
  UART4_Transmit(cmd, 2); // Restart script

  cmd[0] = CMD__GET_SUBROUTINE_SCRIPT_STATUS; // change command
  do {
    UART4_Transmit(cmd, 1); // Get script status
    UART4_Receive(&scriptRunningState, 1);
  } while (!(scriptRunningState & SCRIPT_STATE__STOPPED));

  /*
    uint8_t isMovingCMD = CMD__MOVING_STATE;
    uint8_t isMovingState = MOVING_STATE__MOVING;

    UART4_Transmit(&isMovingCMD, 1);
    do {
      UART4_Transmit(&isMovingState, 1); // Poll for moving state
    } while (isMovingState & MOVING_STATE__MOVING); // Verify is done moving
  */
}

/**
 * @brief
 * @param
 * @return
 */
uint16_t Maestro_GetErrors(void) {
  uint8_t cmd = CMD__GET_ERRORS;
  uint8_t response[2] = {0};

  UART4_Transmit(&cmd, 1);    // Get error status
  UART4_Receive(response, 2); // Read response

  return (uint16_t)((response[1] << 8) | response[0]);
}
