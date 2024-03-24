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
  int8_t idx = 0;
  float  sequences[][Maestro_Channels] = {
      {0.0f,   10.0f, 25.0f,  35.0f,  60.0f,  -23.0f},
      {-15.0f, 19.0f, 10.0f,  -35.0f, -45.0f, 90.0f },
      {-90.0f, -1.0f, 9.0f,   80.0f,  5.0f,   15.0f },
      {5.0f,   20.0f, -90.0f, -75.0f, 0.0f,   72.0f },
      {30.0f,  45.0f, -20.0f, -45.0f, 10.0f,  -45.0f},
  };

  PLL_Init();
  SysTick_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  Maestro_Init(80e6);

  /*
    // Loop channel 0 from -90 to 90
    int8_t diff = 30;
    int8_t target = 90;
    float  angle = 0.0f;

    while (1) {
      for (; idx != target; idx += diff) {
        Maestro_SetAngle(0, (float)idx);
        angle = Maestro_GetPosition(0);
        Maestro_WaitForIdle();
        SysTick_Wait10ms(50);
      }

      diff *= -1;
      target *= -1;
    }
  */

  while (1) {
    for (idx = 0; idx < Maestro_Channels; idx++) {
      Maestro_SetAngles(sequences[idx]);
      SysTick_Wait10ms(50);
    }
    Maestro_WaitForIdle();
  }
}
