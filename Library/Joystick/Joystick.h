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

#include "Quaternion/Quaternion.h"

#define VRx_PIN (1 << 1) // PD1 (AIN6)
#define VRx_AIN 6        // AIN6
#define VRy_PIN (1 << 2) // PD2 (AIN5)
#define VRy_AIN 5        // AIN5

#define PINS   (unsigned)(VRx_PIN | VRy_PIN)
#define PCTL   (unsigned)(GPIO_PCTL_PD1_AIN6 | GPIO_PCTL_PD2_AIN5)
#define PCTL_M (unsigned)(GPIO_PCTL_PD1_M | GPIO_PCTL_PD2_M)

#define JOYSTICK_INT_PRIORITY 2

extern volatile bool HasNewJoystickCoords;

/**
 * @brief
 * @param SYS_CLOCK
 * @param SAMPLING_FREQ
 * @param quatDest
 * @param hasNewData
 */
void Joystick_Init(uint32_t SYS_CLOCK, uint16_t SAMPLING_FREQ, volatile Quaternion *quatDest, volatile bool *hasNewData);

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
