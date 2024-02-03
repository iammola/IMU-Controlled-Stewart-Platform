#include <stdbool.h>
#include <string.h>

#include "PLL.h"
#include "tm4c123gh6pm.h"

#include "UART.h"
#include "JDY-10M.h"

uint8_t myLEDIdx = 0;
uint8_t peerLEDIdx = 0;

static void PortF_Init(void);
static void SetLED(void);

static uint32_t baudRates[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115000, 115200, 230400};
static char *commands[] = {
		"AT",
		"AT+VER",
		"AT+BAUD",
		"AT+PASS",
		"AT+NAME",
		"AT+HOSTEN3",
		"AT\r\n",
		"AT+VER\r\n",
		"AT+BAUD\r\n",
		"AT+PASS\r\n",
		"AT+NAME\r\n",
		"AT+HOSTEN3\r\n",
};

static void PortF_Init(void)
{
	volatile uint32_t delay;
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // activate clock for Port F
	delay = SYSCTL_RCGCGPIO_R;							 // allow time for clock to start

	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock GPIO Port F
	GPIO_PORTF_CR_R = 0x0F;						 // allow changes to PF0-3

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
	GPIO_PORTF_IM_R |= 0x01;	 // Allow interrupts to be sent

	// Enable Interrupt 30 for GPIO Port F
	NVIC_EN0_R |= NVIC_EN0_INT30;

	// Set Priority to one more than UART's for less importance
	NVIC_PRI7_R = (NVIC_PRI7_R & ~NVIC_PRI7_INT30_M) | ((UART_INTERRUPT_PRIORITY + 1) << NVIC_PRI7_INT30_S);
}

static void SetLED(void)
{
	// Turn RED on, then BLUE, then GREEN, then OFF.
	GPIO_PORTF_DATA_R = 0x0E & (1 << myLEDIdx);
}

void UART4_Handler(void)
{
	UART_Receive(&myLEDIdx, 1);
	UART4_ICR_R |= UART_ICR_RXIC | UART_ICR_RTIC;
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
		UART_Transmit(&peerLEDIdx, 1);
	}
}

int main(void)
{
	uint32_t time = 0;
	uint8_t baudIdx = 0;
	uint8_t cmdIdx = 0;

	// Initialize PLL
	PLL_Init();

	// Initialize Port F
	PortF_Init();

	// Initialize UART for a 80MHz, 115200 baud, 8 bit data length, one-eighth FIFO RX interrupts,
	// 1 stop bit, and no parity
	// UART_Init(80e6, 115200, 3 /* UART_LCRH_WLEN_8 */, 0 /* UART_IFLS_RX1_8 */, 0x00 /* No Parity */, false);

#ifndef WIRED
	JDY10M_Init();

	for (baudIdx = 0; baudIdx < (sizeof(baudRates) / sizeof(baudRates[0])); baudIdx++)
	{
		UART_Init(80e6, baudRates[baudIdx], 3 /* UART_LCRH_WLEN_8 */, 4 /* UART_IFLS_RX7_8 */, 0 /* No Parity */, false);

		// UART_Transmit((unsigned char *)"vf hyregkhtog;rj,gx8o7ygnhc x7gdfyg8yg7sfhdenfp9waefrh8o7HDBS*(&^ITEDoy8aehdjhjhbsdx", strlen("vf hyregkhtog;rj,gx8o7ygnhc x7gdfyg8yg7sfhdenfp9waefrh8o7HDBS*(&^ITEDoy8aehdjhjhbsdx"));
		for (cmdIdx = 0; cmdIdx < (sizeof(commands) / sizeof(commands[0])); cmdIdx++)
		{
			UART_Transmit((unsigned char *)commands[cmdIdx], strlen(commands[cmdIdx]));
		}

		while ((UART4_FR_R & UART_FR_BUSY) || (++time < 1e6))
		{
		}

		time = 0;
	}
#endif

	while (1)
		;
}
