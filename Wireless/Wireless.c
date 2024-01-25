#include <stdbool.h>

#include "PLL.h"
#include "tm4c123gh6pm.h"

#include "UART.h"

uint8_t peerIdx = 0;
uint8_t myIdx = 0;

static void PortF_Init(void);
static void SetLED(uint8_t bit);

static void PortF_Init(void)
{
	volatile uint32_t delay;
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // activate clock for Port F
	delay = SYSCTL_RCGCGPIO_R;							 // allow time for clock to start

	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock GPIO Port F
	GPIO_PORTF_CR_R = 0x0F;						 // allow changes to PF0-4

	// enable PF0 (SW2) as inputs and PF1-3 (RBG) as outputs
	GPIO_PORTF_DIR_R = (GPIO_PORTF_DIR_R & ~0x0F) | 0x0E;

	GPIO_PORTF_PUR_R = 0x01;			// enable pull-up on PF0
	GPIO_PORTF_DEN_R |= 0x0F;			// enable digital I/O on PF0-3
	GPIO_PORTF_AMSEL_R &= ~0x0F;	// disable analog on PF0-3
	GPIO_PORTF_AFSEL_R &= ~0x0F;	// disable alternate functions on PF0-3
	GPIO_PORTF_PCTL_R &= ~0xFFFF; // no peripheral functions with disabled alt. functions

	GPIO_PORTF_IS_R &= ~0x01;	 // Configure for Edge-Detect on PF0
	GPIO_PORTF_IBE_R &= ~0x01; // Allow GPIOIEV register to control interrupt
	GPIO_PORTF_IEV_R &= ~0x01; // Trigger falling edge on PF0
	GPIO_PORTF_ICR_R |= 0x01;	 // Clear the interrupt state pin by setting a 1 at the pin
	GPIO_PORTF_IM_R |= 0x01;	 // Allow interrupts to be sent

	// Enable Interrupt 30 for GPIO Port F
	NVIC_EN0_R |= NVIC_EN0_INT30;

	// Set Priority to one less than UART's
	NVIC_PRI7_R = (NVIC_PRI7_R & ~((unsigned)NVIC_PRI7_INT30_M)) | ((UART_INTERRUPT_PRIORITY - 1) << NVIC_PRI7_INT30_S);
}

static void SetLED(uint8_t bit)
{
	// Turn RED on, then BLUE, then GREEN, then OFF.
	GPIO_PORTF_DATA_R = 0x0E & (1 << bit);
}

void UART0_Handler(void)
{
	if (UART0_MIS_R & UART_MIS_RXMIS)
	{
		myIdx = UART_ReceiveByte(1);

		SetLED(myIdx);

		UART0_ICR_R |= UART_ICR_RXIC;
	}
}

void GPIOPortF_Handler(void)
{
	// Confirm the interrupt is caused by SW2
	bool isSW2Interrupt = (GPIO_PORTF_MIS_R & 0x01) == 0x01;

	if (isSW2Interrupt)
	{
		// Update and keep index in bounds
		if (++peerIdx > 3)
			peerIdx = 0;

		// Transmit index
		UART_TransmitByte(peerIdx, 1);

		// Clear the interrupt
		GPIO_PORTF_ICR_R |= 0x01;
	}
}

int main(void)
{
	// Initialize PLL
	PLL_Init();

	// Initialize Port F
	PortF_Init();

	// Initialize UART for a 80MHz, 9600 baud, no High Speed, 8 bit data length,
	// one-eighth FIFO RX interrupts, 1 stop bit, and even parity
	UART_Init(
			80,
			false,
			9600,
			3, // UART_LCRH_WLEN_8
			0, // UART_IFLS_RX1_8
			false,
			true);
}
