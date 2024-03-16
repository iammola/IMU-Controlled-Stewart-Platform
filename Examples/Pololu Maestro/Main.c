/**
 * @file Main.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stdint.h>
#include <string.h>

#include "FPU/fpu.h"
#include "PLL/PLL.h"
#include "SysTick/SysTick.h"

#include "Pololu Maestro/Maestro.h"

int main(void) {
  float sequence1[] = {0.0f, 10.0f, 25.0f, 35.0f, 60.0f, -23.0f};
  float sequence2[] = {-15.0f, 19.0f, 10.0f, -35.0f, -45.0f, 90.0f};
  float sequence3[] = {-90.0f, -1.0f, 9.0f, 80.0f, 5.0f, 15.0f};
  float sequence4[] = {5.0f, 20.0f, -90.0f, -75.0f, 0.0f, 72.0f};
  float sequence5[] = {30.0f, 45.0f, -20.0f, -45.0f, 10.0f, -45.0f};

  PLL_Init();
  SysTick_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  Maestro_Init(80e6);

  while (1) {
    Maestro_SetAngles(sequence1);
    SysTick_Wait10ms(50);
    Maestro_SetAngles(sequence2);
    SysTick_Wait10ms(50);
    Maestro_SetAngles(sequence3);
    SysTick_Wait10ms(50);
    Maestro_SetAngles(sequence4);
    SysTick_Wait10ms(50);
    Maestro_SetAngles(sequence5);
    SysTick_Wait10ms(50);
  }
}
