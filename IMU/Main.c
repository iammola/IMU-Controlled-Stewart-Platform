#include <stdint.h>
#include <stdio.h>

#include "PLL.h"
#include "SysTick.h"

#include "CLI/CLI.h"
#include "IMU.h"

#define SYS_CLOCK   80e6
#define CLI_TXT_BUF 500

static char text[CLI_TXT_BUF] = "";

void Delay(uint32_t inSeconds, int32_t powerOf10);
void Delay(uint32_t inSeconds, int32_t powerOf10) {
  uint32_t CLOCK = SYS_CLOCK;

  while (powerOf10 < 0) {
    CLOCK /= 10;
    ++powerOf10;
  }

  SysTick_Wait(inSeconds * CLOCK);
}

int main(void) {

  Coords accel = {0};
  Coords gyro = {0};
  Coords mag = {0};

  DELAY_FUNC Delay_Func = &Delay;

  SysTick_Init(); // Initialize SysTick
  PLL_Init();     // Initialize the PLL

  CLI_Init(SYS_CLOCK, 115200, WORD_8_BIT, RX_FIFO_7_8, NO_PARITY, ONE_STOP_BIT); // Init UART COM

  IMU_Init(SYS_CLOCK, 1e6, Delay_Func); // Initialize the IMU with the SYS_CLOCK and to communicate at 1MHz

  while (1) {
    IMU_GetGyroReadings(&accel);
    IMU_GetAccelReadings(&gyro);
    IMU_GetMagReadings(&mag);

    snprintf(text, CLI_TXT_BUF, " Accel (X = %d, Y = %d)\n", accel.x, accel.y);
    CLI_Write(text);

    snprintf(text, CLI_TXT_BUF, " Gyro (X = %d, Y = %d)\n", gyro.x, gyro.y);
    CLI_Write(text);

    snprintf(text, CLI_TXT_BUF, " Mag (X = %d, Y = %d)\n", mag.x, mag.y);
    CLI_Write(text);
    
    CLI_Write("\n\r");

    SysTick_Wait10ms(500);
  }
}
