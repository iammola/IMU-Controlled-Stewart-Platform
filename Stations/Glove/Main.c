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
#include "Timer/Ping/Ping.h"
#include "Timer/WaitFor/WaitFor.h"
#include "Wireless/Wireless.h"

#include "Glove.h"

#include "tm4c123gh6pm.h"

#define BTN_BIT                (unsigned)(1 << 0) // PD6
#define BTN_INTERRUPT_PRIORITY (unsigned)4
#define BTN_PCTL_M             (unsigned)(GPIO_PCTL_PD0_M)

#define SYS_CLOCK 80e6

#define IMU_SAMPLE_RATE 300
/* To PC */
// #define SAMPLES_BEFORE_PING ((15 * IMU_SAMPLE_RATE) / 100)
// #define PING_FREQUENCY      100

/* To MCU */
#define SAMPLES_BEFORE_PING ((50 * IMU_SAMPLE_RATE) / 100)
#define PING_FREQUENCY      30

void GPIOD_Handler(void);
void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

static void Button_Init(void);

static void Glove_UpdateControlMethod(MAZE_CONTROL_METHOD newControl);

static volatile Position            position = {0};
static volatile Quaternion          quaternionOffset;
static volatile MAZE_CONTROL_METHOD ExpectingCTLMethod = DEFAULT_CTL_METHOD;
static volatile MAZE_CONTROL_METHOD CTL_METHOD;

void GPIOD_Handler(void) {
  GPIO_PORTD_ICR_R |= BTN_BIT; // clear interrupt

  if ((GPIO_PORTD_DATA_R & BTN_BIT) == 0x00)
    // Try throttle interrupts by checking data register
    return;

  Ping_TimerDisable(); // Disable Ping if enabled
  IMU_Disable();       // Disable IMU

  ExpectingCTLMethod = CTL_METHOD == JOYSTICK_CTL_METHOD ? IMU_CTL_METHOD : JOYSTICK_CTL_METHOD;
  Wireless_Transmit(CHANGE_CONTROL_METHOD, (uint8_t *)&ExpectingCTLMethod, CHANGE_CONTROL_METHOD_LENGTH);

  WaitFor(SYS_CLOCK / 25e-3); // Start Timer to wait for a max of 25ms

  while (!WaitFor_IsDone()) {
    if (ReceivedCommands.ChangeControlMethodAck.isNew) {
      WaitFor_Stop();
      return;
    }
  }

  IMU_Enable(false); // Reaching this point means the ACK was not received
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
  GPIO_PORTD_PDR_R |= BTN_BIT;      // Pull input down
  GPIO_PORTD_DIR_R &= ~BTN_BIT;     // Configure as input
  GPIO_PORTD_AMSEL_R &= ~BTN_BIT;   // Disable Analog mode
  GPIO_PORTD_AFSEL_R &= ~BTN_BIT;   // Disable Alternate functions
  GPIO_PORTD_PCTL_R &= ~BTN_PCTL_M; // Clear peripheral function

  GPIO_PORTD_ICR_R |= BTN_BIT;  // Clear the INT pin's interrupt
  GPIO_PORTD_IM_R &= ~BTN_BIT;  // Disable the INT pin interrupt
  GPIO_PORTD_IS_R &= ~BTN_BIT;  // Configure for Edge-Detect interrupts
  GPIO_PORTD_IBE_R &= ~BTN_BIT; // Only listen on one edge event
  GPIO_PORTD_IEV_R |= BTN_BIT;  // Trigger interrupt on rising edge

  NVIC_EN0_R |= NVIC_EN0_INT3;                                                                              // Enable Port D's Interrupt Handler
  NVIC_PRI0_R = (NVIC_PRI0_R & (unsigned)~NVIC_PRI0_INT3_M) | (BTN_INTERRUPT_PRIORITY << NVIC_PRI0_INT3_S); // Configure Port D's priority

  GPIO_PORTD_IM_R |= BTN_BIT; // Allow the INT pin interrupt to be detected
}

static void Glove_UpdateControlMethod(MAZE_CONTROL_METHOD newControl) {
  if (ExpectingCTLMethod != newControl)
    return;

  CTL_METHOD = newControl;

  switch (CTL_METHOD) {
    case IMU_CTL_METHOD:
      IMU_Enable(true);
      break;
    case JOYSTICK_CTL_METHOD:
      IMU_Disable();
      break;
  }
}

inline void Ping_Handler(void) {
  Ping_TimerDisable();

  position.inUse = true;

  // Get the diff from the offset quaternion and scale it down
  position.quaternion = QuaternionMultiply(position.quaternion, quaternionOffset, 0.5f);

  Wireless_Transmit(NEW_POSITION, (uint8_t *)&position, NEW_POSITION_LENGTH);

  position.inUse = false;
  position.count = 0;
}

int main(void) {
  DisableInterrupts();

  quaternionOffset = QuaternionConjugate(0.615685642f, 0.688054919f, -0.0159532707f, -0.384057611f);

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  Wireless_Init(SYS_CLOCK, true);
  IMU_Init(SYS_CLOCK, IMU_SAMPLE_RATE, &position);
  Ping_TimerInit(SYS_CLOCK / PING_FREQUENCY, true);

  Button_Init();

  EnableInterrupts();

  while (1) {
    WaitForInterrupt();

    if (CTL_METHOD == IMU_CTL_METHOD && position.count == SAMPLES_BEFORE_PING) {
      Ping_TimerEnable();
    }

    if (ReceivedCommands.ChangeControlMethodAck.isNew) {
      ReceivedCommands.ChangeControlMethodAck.inUse = true;

      Glove_UpdateControlMethod(ReceivedCommands.ChangeControlMethodAck.data[0]);

      ReceivedCommands.ChangeControlMethodAck.isNew = ReceivedCommands.ChangeControlMethodAck.inUse = false;
    }
  }
}
