#include <stdint.h>
#include <stdbool.h>

#include "PLL.h"
#include "tm4c123gh6pm.h"

#include "RFM69HCW.h"

// Tracked LED for peer node
static uint8_t peerLEDIdx = 0;

static void PortF_Init(void)
{
  volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // activate clock for Port F
  delay = SYSCTL_RCGCGPIO_R;               // allow time for clock to start

  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x0F;            // allow changes to PF0-3

  // enable PF0 (SW2) as inputs and PF1-3 (RBG) as outputs
  GPIO_PORTF_DIR_R = (GPIO_PORTF_DIR_R & ~0x0F) | 0x0E;

  GPIO_PORTF_PUR_R = 0x01;      // enable pull-up on PF0
  GPIO_PORTF_DEN_R |= 0x0F;     // enable digital I/O on PF0-3
  GPIO_PORTF_AMSEL_R &= ~0x0F;  // disable analog on PF0-3
  GPIO_PORTF_AFSEL_R &= ~0x0F;  // disable alternate functions on PF0-3
  GPIO_PORTF_PCTL_R &= ~0xFFFF; // no peripheral functions with disabled alt. functions

  GPIO_PORTF_IS_R &= ~0x01;  // Configure for Edge-Detect on PF0
  GPIO_PORTF_IBE_R &= ~0x01; // Allow GPIOIEV register to control interrupt
  GPIO_PORTF_IEV_R &= ~0x01; // Trigger falling edge on PF0
  GPIO_PORTF_IM_R |= 0x01;   // Allow interrupts to be sent

  // Enable Interrupt 30 for GPIO Port F
  NVIC_EN0_R |= NVIC_EN0_INT30;

  // Set Priority to one more than UART's for less importance
  NVIC_PRI7_R = (NVIC_PRI7_R & ~NVIC_PRI7_INT30_M) | ((RFM69HCW_INT_PRIORITY + 1) << NVIC_PRI7_INT30_S);
}

static void SetLED(uint8_t data)
{
  // Turn RED on, then BLUE, then GREEN, then OFF.
  GPIO_PORTF_DATA_R = 0x0E & (1 << data);
}

void GPIOF_Handler(void)
{
  if (GPIO_PORTF_MIS_R & 0x01)
  {
    // Clear the interrupt
    GPIO_PORTF_ICR_R |= 0x01;

    // Update and keep index in bounds
    peerLEDIdx = (peerLEDIdx + 1) & 3;

    // Transmit index
    RFM69HCW_SendPacket(&peerLEDIdx, 1);
  }
}

int main(void)
{
  // Initialize PLL
  PLL_Init();

  // Initialize Port F
  PortF_Init();

  // Initialize RFM69HCW Wireless Module
  RFM69HCW_Init(80e6, 1e6);

  while (1)
  {
    // Wait for new data to be confirmed
    if (HasNewData)
    {
      // Toggle LED with data received
      SetLED(RX_Data_Buffer[0]);

      // Clear data flag
      HasNewData = false;
    }
  }
}
