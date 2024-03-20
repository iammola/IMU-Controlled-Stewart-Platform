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

#include <stdint.h>

#include "tm4c123gh6pm.h"

#define VRx_PIN (1 << 0) // PE0 (AIN3)
#define VRx_AIN 3        // AIN3
#define VRy_PIN (1 << 1) // PE1 (AIN2)
#define VRy_AIN 2        // AIN2
#define SW_PIN  (1 << 2) // PE2

#define PINS   (unsigned)(VRx_PIN | VRy_PIN | SW_PIN)
#define AFSEL  (unsigned)(VRx_PIN | VRy_PIN)
#define PCTL   (unsigned)(GPIO_PCTL_PE0_AIN3 | GPIO_PCTL_PE1_AIN2)
#define PCTL_M (unsigned)(GPIO_PCTL_PE0_M | GPIO_PCTL_PE1_M | GPIO_PCTL_PE2_M)

#define JOYSTICK_INT_PRIORITY 2

/**
 * @brief
 * @param SYS_CLOCK
 * @param SAMPLING_FREQ
 */
void Joystick_Init(uint32_t SYS_CLOCK, uint16_t SAMPLING_FREQ);
