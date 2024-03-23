/**
 * @file Joystick.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "tm4c123gh6pm.h"

#include "POSITION.h"
#include "Quaternion/Quaternion.h"

#define V1Rx_PIN (1 << 0) // PD0 (AIN7)
#define V1Rx_AIN 7        // AIN7
#define V1Ry_PIN (1 << 1) // PD1 (AIN6)
#define V1Ry_AIN 6        // AIN6

#define V2Rx_PIN (1 << 2) // PD2 (AIN5)
#define V2Rx_AIN 5        // AIN5
#define V2Ry_PIN (1 << 3) // PD3 (AIN4)
#define V2Ry_AIN 4        // AIN4

#define PINS   (unsigned)(V1Rx_PIN | V1Ry_PIN)
#define PCTL   (unsigned)(GPIO_PCTL_PD0_AIN7 | GPIO_PCTL_PD1_AIN6 | GPIO_PCTL_PD2_AIN5 | GPIO_PCTL_PD3_AIN4)
#define PCTL_M (unsigned)(GPIO_PCTL_PD0_M | GPIO_PCTL_PD1_M | GPIO_PCTL_PD2_M | GPIO_PCTL_PD3_M)

#define JOYSTICK_INT_PRIORITY 2

#define PAN_RANGE 30.0f

extern volatile bool HasNewJoystickCoords;

/**
 * @brief
 * @param SYS_CLOCK
 * @param SAMPLING_FREQ
 * @param position
 */
void Joystick_Init(uint32_t SYS_CLOCK, uint16_t SAMPLING_FREQ, volatile Position *position);

/**
 * @brief
 * @param
 */
void Joystick_Enable(void);

/**
 * @brief
 * @param
 */
void Joystick_Disable(void);
