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

#include "IMU/IMU.h"
#include "Joystick/Joystick.h"
#include "Pololu Maestro/Maestro.h"
#include "StewartPlatform/StewartPlatform.h"

#define SYS_CLOCK    80e6
#define INT_PRIORITY 3

#define DURATION 7000.0f

void WaitForInterrupt(void);

void TIMER0A_Handler(void);
void Timer_Init(uint32_t FREQ);

static volatile float    elapsed = 0.0f;
static volatile Position position = {0};

/**
 * @brief Stewart Platform "Tilt" Animation
 * @link https://github.com/rawify/Stewart.js/blob/961177ccb21a9dbb22b54393f991f315925f5a52/stewart.js#L592-L624
 * @param
 */
void TIMER0A_Handler(void) {
  float elapsedInPercent = 0.0f;

  float angle = 0.0f;
  float a = 0.0f;
  float x = 0.0f, y = 0.0f, z = 0.0f;

  if (elapsed++ == DURATION)
    elapsed = 0.0f;

  elapsedInPercent = elapsed / DURATION;

  if (elapsedInPercent < (1.0f / 4.0f)) {
    a = 0.0f;
  } else if (elapsedInPercent < (1.0f / 2.0f)) {
    elapsedInPercent -= 1.0f / 4.0f;
    a = M_PI / 3.0f;
  } else if (elapsedInPercent < (3.0f / 4.0f)) {
    elapsedInPercent -= 1.0f / 2.0f;
    a = (2 * M_PI) / 3.0f;
  } else {
    elapsedInPercent -= 3.0f / 4.0f;
    z = 1.0f;
  }

  elapsedInPercent *= 4.0f;

  if (z == 0.0f) {
    x = sinf(a);
    y = -cosf(a);
  }

  angle = powf(sinf((elapsedInPercent * M_PI * 2) - (M_PI * 8)), 5) / 3.0f;

  // position.translation = ((Coords){0});
  if (!position.inUse) {
    position.quaternion = QuaternionFromAxisAngle(x, y, z, angle);
  }

  TIMER0_ICR_R |= TIMER_ICR_TATOCINT; // Clear interrupt
}

void Timer_Init(uint32_t FREQ) {
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0; // Enable Timer Module 0

  TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;      // Periodic timer
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;            // Trigger interrupt when reaches limit
  TIMER0_TAILR_R = (uint32_t)SYS_CLOCK / FREQ; // Set Load value

  NVIC_EN0_R |= NVIC_EN0_INT19;                                                             // Enable Timer 0 SubTimer A Interrupt Handler
  NVIC_PRI14_R = (NVIC_PRI14_R & ~NVIC_PRI4_INT19_M) | (INT_PRIORITY << NVIC_PRI4_INT19_S); // Set Priority
  TIMER0_CTL_R |= TIMER_CTL_TAEN;                                                           // Enable Timer A
}

int main(void) {
  uint8_t legIdx = 0;

  PLL_Init();              // Initialize the PLL
  FPULazyStackingEnable(); // Enable Floating Point

  Maestro_Init(SYS_CLOCK); // Initialize Maestro Controller
  StewartPlatform_Init();  // Initialize stewart platform
  Timer_Init(1500);

  for (legIdx = 0; legIdx < Maestro_Channels; legIdx++) {
    Maestro_SetAngle(legIdx, legs[legIdx].servoAngle);
  }

  while (1) {
    WaitForInterrupt();

    position.inUse = true;

    StewartPlatform_Update(position.translation, position.quaternion);
    for (legIdx = 0; legIdx < Maestro_Channels; legIdx++) {
      Maestro_SetAngle(legIdx, legs[legIdx].servoAngle);
    }

    position.inUse = false;
    Maestro_WaitForIdle();
  }
}
