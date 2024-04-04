#include <stdbool.h>
#include <stdint.h>

#include "PLL/PLL.h"
#include "SysTick/SysTick.h"

#include "tm4c123gh6pm.h"

#include "Wireless/Wireless.h"

#define SYS_CLOCK 80e6

#define SW2_BIT        (unsigned)0x01
#define SW1_BIT        (unsigned)0x10
#define LED_BITS       (unsigned)0x0E
#define PINS           (unsigned)(SW2_BIT | LED_BITS | SW1_BIT)
#define PINS_PCTL_MASK (unsigned)(GPIO_PCTL_PF0_M | GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M)

void GPIOF_Handler(void);

// Tracked LED for peer node
static uint8_t       peerLEDIdx = 0;
static volatile bool SendData = false;

static void PortF_Init(void) {
  volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // activate clock for Port F
  delay = SYSCTL_RCGCGPIO_R;               // allow time for clock to start

  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x0F;            // allow changes to PF0-3

  // enable PF0 (SW2) as inputs and PF1-3 (RBG) as outputs
  GPIO_PORTF_DIR_R = (GPIO_PORTF_DIR_R & ~PINS) | LED_BITS;

  GPIO_PORTF_PUR_R |= SW2_BIT | SW1_BIT; // enable pull-up on PF0
  GPIO_PORTF_DEN_R |= PINS;              // enable digital I/O on PF0-3
  GPIO_PORTF_AMSEL_R &= ~PINS;           // disable analog on PF0-3
  GPIO_PORTF_AFSEL_R &= ~PINS;           // disable alternate functions on PF0-3
  GPIO_PORTF_PCTL_R &= ~PINS_PCTL_MASK;  // no peripheral functions with disabled alt. functions

  GPIO_PORTF_IS_R &= ~(SW2_BIT | SW1_BIT);  // Configure for Edge-Detect on PF0
  GPIO_PORTF_IBE_R &= ~(SW2_BIT | SW1_BIT); // Allow GPIOIEV register to control interrupt
  GPIO_PORTF_IEV_R &= ~(SW2_BIT | SW1_BIT); // Trigger falling edge on PF0
  GPIO_PORTF_IM_R |= (SW2_BIT | SW1_BIT);   // Allow interrupts to be sent

  NVIC_EN0_R |= NVIC_EN0_INT30;                                                          // Enable Interrupt 30 for GPIO Port F
  NVIC_PRI7_R = (NVIC_PRI7_R & (unsigned)~NVIC_PRI7_INT30_M) | (6 << NVIC_PRI7_INT30_S); // Set Priority
}

void GPIOF_Handler(void) {
  if (GPIO_PORTF_MIS_R & SW2_BIT) {
    SendData = true;
    GPIO_PORTF_ICR_R |= SW2_BIT;
  } else if (GPIO_PORTF_MIS_R & SW1_BIT) {
    SendData = false;
    GPIO_PORTF_ICR_R |= SW1_BIT;
  }
}

int main(void) {
  SysTick_Init(); // Initialize SysTick
  PLL_Init();     // Initialize PLL
  PortF_Init();   // Initialize Port F

  Wireless_Init(SYS_CLOCK, true);

  while (1) {
    // Wait for new data to be confirmed
    if (ReceivedCommands.NewPosition.inUse) {
      GPIO_PORTF_DATA_R = LED_BITS & (1 << ReceivedCommands.NewPosition.data[0]); // Turn RED on, then BLUE, then GREEN, then OFF.
      ReceivedCommands.NewPosition.inUse = false;                                 // Clear data flag
    }

    if (SendData) {
      peerLEDIdx = (peerLEDIdx + 1) & 3;               // Update and keep index in bounds
      Wireless_Transmit(NEW_POSITION, &peerLEDIdx, 1); // Transmit index

      SysTick_Wait10ms(200); // Wait 2s
    }
  }
}
