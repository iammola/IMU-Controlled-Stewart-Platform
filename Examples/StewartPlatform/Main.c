/**
 * @file Main.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <stdint.h>
#include <stdio.h>

#include "FPU/fpu.h"
#include "PLL/PLL.h"
#include "SysTick/SysTick.h"
#include "tm4c123gh6pm.h"

#include "Pololu Maestro/Maestro.h"
#include "StewartPlatform/StewartPlatform.h"

#define SYS_CLOCK 80e6

#define ANIMATION 0 // 0 = Tilt, 1 = Wobble

static const uint32_t DURATION = (uint32_t)(SYS_CLOCK * (ANIMATION == 1 ? 3 : 10));

int main(void) {
  Quaternion orientation = {0};
  Coords     translation = {0};

  uint8_t  legIdx = 0;
  uint32_t elapsed = 0;

  float angle = 0.0f;
  float elapsedRatio = 0.0f;
#if ANIMATION == 0 // Tilt
  float a = 0.0f, x = 0.0f, y = 0.0f, z = 0.0f;
#endif

  SysTick_Init();          // Initialize SysTick
  PLL_Init();              // Initialize the PLL
  FPULazyStackingEnable(); // Enable Floating Point

  Maestro_Init(SYS_CLOCK); // Initialize Maestro Controller
  StewartPlatform_Init();  // Initialize stewart platform

  for (legIdx = 0; legIdx < Maestro_Channels; legIdx++) {
    Maestro_SetAngle(legIdx, legs[legIdx].servoAngle);
  }

  while (1) {
    for (elapsed = 0; elapsed <= DURATION; elapsed += (ST_RELOAD_R - ST_CURRENT_R)) {
      ST_CURRENT_R = 0; // reset tick counter

      elapsedRatio = (float)elapsed / (float)DURATION;

// https://github.com/rawify/Stewart.js/blob/961177ccb21a9dbb22b54393f991f315925f5a52/stewart.js#L574-L774
#if ANIMATION == 0 // Tilt
      x = 0.0f;
      y = 0.0f;
      z = 0.0f;

      if (elapsedRatio < (1.0f / 4.0f)) {
        a = 0.0f;
      } else if (elapsedRatio < (1.0f / 2.0f)) {
        elapsedRatio -= 1.0f / 4.0f;
        a = M_PI / 3.0f;
      } else if (elapsedRatio < (3.0f / 4.0f)) {
        elapsedRatio -= 1.0f / 2.0f;
        a = (2 * M_PI) / 3.0f;
      } else {
        elapsed = 0;
        continue; // Z-rotation doesn't work great
        elapsedRatio -= 3.0f / 4.0f;
        z = 1.0f;
      }

      elapsedRatio *= 4.0f;

      if (z == 0.0f) {
        x = sinf(a);
        y = -cosf(a);
      }

      angle = powf(sinf((elapsedRatio * M_PI * 2) - (M_PI * 8)), 5) / 3.0f;

      translation = (Coords){0};
      orientation = QuaternionFromAxisAngle(x, y, z, angle);
#elif ANIMATION == 1 // Wobble
      angle = elapsedRatio * M_PI * 2.0f;

      translation.x = cosf(-angle) * 13.0f;
      translation.y = sinf(-angle) * 13.0f;
      translation.z = 0.0f;

      orientation = normalizeQuaternion(-13.0f, -cosf(angle), sinf(angle), 0);
#endif

      StewartPlatform_Update(translation, orientation);
      for (legIdx = 0; legIdx < Maestro_Channels; legIdx++) {
        Maestro_SetAngle(legIdx, legs[legIdx].servoAngle * ((legIdx % 2 == 0) ? 1.0f : -1.0f));
      }

      Maestro_WaitForIdle();
    }
  }
}
