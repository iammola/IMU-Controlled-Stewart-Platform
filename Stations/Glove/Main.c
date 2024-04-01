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

#include "IMU/IMU.h"
#include "Timer/Ping/Ping.h"
#include "Wireless/Wireless.h"

#include "Glove.h"

#include "tm4c123gh6pm.h"

#define BTN_BIT    (1 << 0) // PD0
#define BTN_PCTL_M (unsigned)(GPIO_PCTL_PD0_M)

#define SYS_CLOCK 80e6

void GPIOD_Handler(void);
void WaitForInterrupt(void);
void EnableInterrupts(void);
void DisableInterrupts(void);

static void Button_Init(void);

static void Enable_Control_Method(uint8_t dataLength, uint8_t *buffer);

static volatile MAZE_CONTROL_METHOD CTL_METHOD = DEFAULT_CTL_METHOD;

static Position   position = {0};
static Quaternion positionFromOffset = {0};
static uint8_t    positionInBytes[NEW_POSITION_LENGTH] = {0};

void GPIOD_Handler(void) {
  uint8_t byte = CTL_METHOD == JOYSTICK_CTL_METHOD ? IMU_CTL_METHOD : JOYSTICK_CTL_METHOD;

  if (!(GPIO_PORTD_MIS_R & BTN_BIT))
    return;

  GPIO_PORTD_ICR_R |= BTN_BIT; // clear interrupt

  Wireless_Transmit(CHANGE_CONTROL_METHOD, &byte, CHANGE_CONTROL_METHOD_LENGTH);
}

void Ping_Handler(void) {
  DisableInterrupts();
  Wireless_Transmit(PING, 0, 0); // Send ping
  EnableInterrupts();
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

  NVIC_EN0_R |= NVIC_EN0_INT3;                                                         // Enable Port D's Interrupt Handler
  NVIC_PRI0_R = (NVIC_PRI0_R & (unsigned)~NVIC_PRI0_INT3_M) | (4 << NVIC_PRI0_INT3_S); // Configure Port D's priority
  GPIO_PORTB_ICR_R |= BTN_BIT;                                                         // Clear the INT pin's interrupt

  GPIO_PORTB_IM_R |= BTN_BIT; // Allow the INT pin interrupt to be detected
}

/**
 * @brief
 * @param dataLength
 * @param buffer
 */
static void Enable_Control_Method(uint8_t dataLength, uint8_t *buffer) {
  if (dataLength != CHANGE_CONTROL_METHOD_LENGTH)
    return;

  CTL_METHOD = buffer[0];

  switch (CTL_METHOD) {
    case IMU_CTL_METHOD:
      IMU_Enable();
      break;
    default:
      IMU_Disable();
      break;
  }
}

int main(void) {
  COMMAND          cmd = 0x00;
  uint8_t          dataLength = 0x00;
  const Quaternion positionOffset = QuaternionInverse(0.628237f, 0.014747f, -0.585671f, -0.512185f);

  DisableInterrupts();

  PLL_Init();
  FPULazyStackingEnable(); // Enable Floating Point

  Ping_TimerInit(SYS_CLOCK);
  Wireless_Init(SYS_CLOCK, false);
  IMU_Init(SYS_CLOCK, &position);

  if (CTL_METHOD == IMU_CTL_METHOD)
    IMU_Enable();

  Button_Init();

  EnableInterrupts();

  while (1) {
    WaitForInterrupt();

    if (position.isNew) {
      // Get the diff from the offset quaternion
      positionFromOffset = QuaternionMultiply(position.quaternion, positionOffset);

      memcpy(positionInBytes, &positionFromOffset, NEW_POSITION_LENGTH);
      Wireless_Transmit(NEW_POSITION, positionInBytes, NEW_POSITION_LENGTH);

      position.isNew = false;
    }

    if (HasNewData) {
      cmd = RX_Data_Buffer[0];
      dataLength = RX_Data_Buffer[1];

      switch (cmd) {
        case CHANGE_CONTROL_METHOD_ACK:
          Enable_Control_Method(dataLength, RX_Data_Buffer + DATA_OFFSET);
          break;
        default:
          break;
      }

      HasNewData = false;
    }
  }
}
