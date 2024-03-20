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
#include <math.h>
#include <stdio.h>

#include "CLI/CLI.h"
#include "FPU/fpu.h"
#include "PLL/PLL.h"

#include "Joystick/Joystick.h"

#define SYS_CLOCK   80e6
#define SAMPLE_RATE 10

#define DIFF 1023.0f

static volatile uint16_t VRx_SAMPLE;
static volatile uint16_t VRy_SAMPLE;

void ADC0SS1_Handler(void);
void ADC0SS1_Handler(void) {
  // Clear SS1 Interrupt
  ADC0_ISC_R |= ADC_ISC_IN1;

  VRx_SAMPLE = (uint16_t)ADC0_SSFIFO1_R; // 1st Sample
  VRy_SAMPLE = (uint16_t)ADC0_SSFIFO1_R; // 2nd Sample
}

static char text[CLI_TXT_BUF] = "";

int main(void) {
  float VRx = 0.0f;
  float VRy = 0.0f;
  float angle = 0.0f;

  PLL_Init();
  FPULazyStackingEnable();                                                       // Enable Floating Point for use especially in Interrupts
  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_OFF, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  Joystick_Init(SYS_CLOCK, SAMPLE_RATE);

  while (1) {
    // Calculate co-ordinates, from -X, -Y (-1) to X, Y (1)
    VRx = (VRx_SAMPLE / 2048.0f) - 1.0f;
    VRy = (VRy_SAMPLE / 2048.0f) - 1.0f;

    angle = atan2f(VRx, VRy);

    snprintf(text, CLI_TXT_BUF, "-13,%0.4f,%0.4f,0", -cosf(angle), sinf(angle));

    CLI_Write(text);
    CLI_Write("\n");
  }
}
