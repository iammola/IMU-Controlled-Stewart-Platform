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

void Maestro_Init(uint32_t SYS_CLOCK);

void Maestro_SetAngles(float angles[Maestro_Channels]);
