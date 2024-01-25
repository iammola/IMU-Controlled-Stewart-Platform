#include <stdbool.h>

#include "PLL.h"

#include "UART.h"

int main(void)
{
	PLL_Init();

	UART_Init(
			16,
			false,
			9600,
			3, // UART_LCRH_WLEN_8
			1, // UART_IFLS_RX2_8
			true,
			true);
}
