#include "Pixy2_1.h"

#include "PLL.h"
#include "SysTick.h"

#define SYS_CLOCK 80e6

#define COLOR_MAX 255

int main(void) {
  uint8_t R = 0;
  uint8_t G = 0;
  uint8_t B = 0;

  PLL_Init();
  SysTick_Init();

  Pixy2_1_Init(SYS_CLOCK);

  for (R = 0; R < COLOR_MAX; R++) {
    for (G = 0; G < COLOR_MAX; G++) {
      for (B = 0; B < COLOR_MAX; B++) {
        Pixy2_1_SetLED(R, G, B); // Loop through all possible colors
        SysTick_Wait10ms(20);    // Wait 200ms between
      }
    }
  }

  while (1) {
  }
}
