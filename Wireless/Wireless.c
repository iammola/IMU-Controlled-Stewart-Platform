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
			3,
			false,
			true,
			true);
}
