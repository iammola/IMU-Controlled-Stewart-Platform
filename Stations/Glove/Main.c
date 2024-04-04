/**
 * @file Main.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <string.h>

#include "FPU/fpu.h"
#include "PLL/PLL.h"
#include "SysTick/SysTick.h"

#include "IMU/IMU.h"
#include "Wireless/Wireless.h"

#include "Glove.h"

#include "tm4c123gh6pm.h"

#define BTN_BIT                (unsigned)(1 << 0) // PD0
#define BTN_INTERRUPT_PRIORITY (unsigned)4
#define BTN_PCTL_M             (unsigned)(GPIO_PCTL_PD0_M)

#define SYS_CLOCK 80e6

void GPIOD_Handler(void);
void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

static void Button_Init(void);

static void Enable_Control_Method(MAZE_CONTROL_METHOD newControl);

static Position                     position = {0};
static volatile MAZE_CONTROL_METHOD ExpectingCTLMethod;
static volatile MAZE_CONTROL_METHOD CTL_METHOD = DEFAULT_CTL_METHOD;

void GPIOD_Handler(void) {
  MAZE_CONTROL_METHOD byte = CTL_METHOD == JOYSTICK_CTL_METHOD ? IMU_CTL_METHOD : JOYSTICK_CTL_METHOD;

  if (!(GPIO_PORTD_MIS_R & BTN_BIT))
    return;

  GPIO_PORTD_ICR_R |= BTN_BIT; // clear interrupt
  Wireless_Transmit(CHANGE_CONTROL_METHOD, (uint8_t *)&byte, CHANGE_CONTROL_METHOD_LENGTH);
  ExpectingCTLMethod = byte;
}

/**
 * @brief
 * @param
 */
static void Button_Init(void) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;                 // Enable Port D
  while ((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R3) == 0x00) { // Wait for Port ready
  }

  GPIO_PORTD_DEN_R |= BTN_BIT;      // Enable Digital
  GPIO_PORTD_PDR_R |= BTN_BIT;      // Configure as active low
  GPIO_PORTD_DIR_R &= ~BTN_BIT;     // Configure as input
  GPIO_PORTD_AMSEL_R &= ~BTN_BIT;   // Disable Analog mode
  GPIO_PORTD_AFSEL_R &= ~BTN_BIT;   // Disable Alternate functions
  GPIO_PORTD_PCTL_R &= ~BTN_PCTL_M; // Clear peripheral function

  GPIO_PORTD_IM_R &= ~BTN_BIT;  // Disable the INT pin interrupt
  GPIO_PORTD_IS_R &= ~BTN_BIT;  // Configure for Edge-Detect interrupts
  GPIO_PORTD_IBE_R &= ~BTN_BIT; // Only listen on one edge event
  GPIO_PORTD_IEV_R |= BTN_BIT;  // Trigger interrupt on rising edge

  NVIC_EN0_R |= NVIC_EN0_INT3;                                                                              // Enable Port D's Interrupt Handler
  NVIC_PRI0_R = (NVIC_PRI0_R & (unsigned)~NVIC_PRI0_INT3_M) | (BTN_INTERRUPT_PRIORITY << NVIC_PRI0_INT3_S); // Configure Port D's priority
  GPIO_PORTD_ICR_R |= BTN_BIT;                                                                              // Clear the INT pin's interrupt

  GPIO_PORTD_IM_R |= BTN_BIT; // Allow the INT pin interrupt to be detected
}

static void Enable_Control_Method(MAZE_CONTROL_METHOD newControl) {
  if (ExpectingCTLMethod != newControl)
    return;

  CTL_METHOD = newControl;

  switch (CTL_METHOD) {
    case IMU_CTL_METHOD:
      IMU_Enable();
      break;
    case JOYSTICK_CTL_METHOD:
      IMU_Disable();
      break;
  }
}

int main(void) {
  static uint32_t count = 0x00;
  const uint32_t  ticks = (60 * 366) / 100;
  // QuaternionConjugate(0.604001f, 0.622846f, -0.294529f, -0.400927f);
  Quaternion quaternionOffset = QuaternionConjugate(0.541299462f, 0.69512105f, 0.122621112f, -0.457175076f);

  DisableInterrupts();

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  Wireless_Init(SYS_CLOCK, true);
  IMU_Init(SYS_CLOCK, &position);

  if (CTL_METHOD == IMU_CTL_METHOD)
    IMU_Enable();

  Button_Init();

  EnableInterrupts();

  while (1) {
    WaitForInterrupt();

    if (position.isNew) {
      if (count == 0) {
        DisableInterrupts();
        position.quaternion = QuaternionMultiply(position.quaternion, quaternionOffset); // Get the diff from the offset quaternion
        Wireless_Transmit(NEW_POSITION, (uint8_t *)&position, NEW_POSITION_LENGTH);
        position.isNew = false;
        EnableInterrupts();
      }

      count = (count + 1) % ticks;
    }

    if (ReceivedCommands.ChangeControlMethodAck.inUse) {
      Enable_Control_Method(ReceivedCommands.ChangeControlMethodAck.data[0]);
      ReceivedCommands.ChangeControlMethodAck.inUse = false;
    }
  }
}
