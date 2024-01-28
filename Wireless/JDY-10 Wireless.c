#ifndef WIRED
#include <stdbool.h>

#include "PLL.h"
#include "tm4c123gh6pm.h"

#include "JDY-10M.h"

int main(void)
{
	// Initialize PLL
	PLL_Init();

	JDY10M_Init(80e6);

	while (1)
		;
}
#endif
