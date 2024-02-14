#include <stdint.h>
#include <stdbool.h>

#include "PLL.h"
#include "tm4c123gh6pm.h"

#include "RFM69HCW.h"

#define SW2_BIT (unsigned)0x01
#define LED_BITS (unsigned)0x0E
#define PINS (unsigned)(SW2_BIT | LED_BITS)
#define PINS_PCTL_MASK (unsigned)(GPIO_PCTL_PF0_M | GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M)

void GPIOF_Handler(void);

// Tracked LED for peer node
static uint8_t peerLEDIdx = 0;

static void PortF_Init(void) {
  volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // activate clock for Port F
  delay = SYSCTL_RCGCGPIO_R;               // allow time for clock to start

  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x0F;            // allow changes to PF0-3

  // enable PF0 (SW2) as inputs and PF1-3 (RBG) as outputs
  GPIO_PORTF_DIR_R = (GPIO_PORTF_DIR_R & ~PINS) | LED_BITS;

  GPIO_PORTF_PUR_R |= SW2_BIT;          // enable pull-up on PF0
  GPIO_PORTF_DEN_R |= PINS;             // enable digital I/O on PF0-3
  GPIO_PORTF_AMSEL_R &= ~PINS;          // disable analog on PF0-3
  GPIO_PORTF_AFSEL_R &= ~PINS;          // disable alternate functions on PF0-3
  GPIO_PORTF_PCTL_R &= ~PINS_PCTL_MASK; // no peripheral functions with disabled alt. functions

  GPIO_PORTF_IS_R &= ~SW2_BIT;  // Configure for Edge-Detect on PF0
  GPIO_PORTF_IBE_R &= ~SW2_BIT; // Allow GPIOIEV register to control interrupt
  GPIO_PORTF_IEV_R &= ~SW2_BIT; // Trigger falling edge on PF0
  GPIO_PORTF_IM_R |= SW2_BIT;      // Allow interrupts to be sent

  // Enable Interrupt 30 for GPIO Port F
  NVIC_EN0_R |= NVIC_EN0_INT30;

  // Set Priority to one more than UART's for less importance
  NVIC_PRI7_R = (NVIC_PRI7_R & (unsigned)~NVIC_PRI7_INT30_M) | ((RFM69HCW_INT_PRIORITY + 1) << NVIC_PRI7_INT30_S);
}

void GPIOF_Handler(void) {
  if (GPIO_PORTF_MIS_R & SW2_BIT) {
    // Clear the interrupt
    GPIO_PORTF_ICR_R |= SW2_BIT;

    // Update and keep index in bounds
    peerLEDIdx = (peerLEDIdx + 1) & 3;

    // Transmit index
    RFM69HCW_SendPacket(&peerLEDIdx, 1, false);
  }
}

int main(void) {
  // Initialize PLL
  PLL_Init();

  // Initialize Port F
  PortF_Init();

  // Initialize RFM69HCW Wireless Module
  RFM69HCW_Init(80e6, 1e6);

  while (1) {
    // Wait for new data to be confirmed
    if (HasNewData) {
      // Turn RED on, then BLUE, then GREEN, then OFF.
      GPIO_PORTF_DATA_R = LED_BITS & (1 << RX_Data_Buffer[0]);

      // Clear data flag
      HasNewData = false;
    }
    else
    {
      RFM69HCW_ReadRegisterCLI();
    }
  }
}
