#include "Pixy2_1.h"

#include "PLL.h"
#include "SysTick.h"

#define SYS_CLOCK 80e6

#define COLOR_MAX 255

int main(void) {
  PLL_Init();
  SysTick_Init();

  Pixy2_1_Init(SYS_CLOCK);

  Pixy2_1_SetLED(174, 23, 100);
}
