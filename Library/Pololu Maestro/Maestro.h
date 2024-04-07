/**
 * @file Maestro.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stdint.h>

#define Maestro_Channels 6

#define Maestro_BAUD 115200

#define PROTOCOL__COMPACT  0xFF
#define PROTOCOL__MINI_SSC 0xEA
#define PROTOCOL           PROTOCOL__MINI_SSC

#define MINI_SSC__OFFSET 0x00

#define CMD__SET_TARGET_MINI_SSC          0xFF
#define CMD__SET_TARGET                   0x84
#define CMD__GET_POSITIONS                0x90
#define CMD__GO_HOME                      0xA2
#define CMD__RESTART_SUBROUTINE_SCRIPT    0xA7
#define CMD__GET_SUBROUTINE_SCRIPT_STATUS 0xAE
#define CMD__MOVING_STATE                 0x93
#define CMD__GET_ERRORS                   0xA1

#define MOVING_STATE__MOVING  0x01
#define SCRIPT_STATE__STOPPED 0x01

#define FULL_ANGLE        90.0f
#define NEUTRAL_PULSE_POS 1500.0f
#define FULL_PULSE_POS    2400.0f
#define K                 (float)((FULL_PULSE_POS - NEUTRAL_PULSE_POS) / FULL_ANGLE)

void Maestro_Init(const uint32_t SYS_CLOCK);

void Maestro_SetAngles(float angles[Maestro_Channels]);

void Maestro_SetAngle(uint8_t channel, float angle);

void Maestro_GetPositions(void);

float Maestro_GetPosition(uint8_t channel);

void Maestro_WaitForIdle(void);

uint16_t Maestro_GetErrors(void);
