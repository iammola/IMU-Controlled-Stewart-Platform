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

#include "UTILS/UTILS.h"

/**
 * @brief
 * @param SYS_CLOCK
 * @param position
 */
void Joystick_Init(uint32_t SYS_CLOCK, volatile Position *position);

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
