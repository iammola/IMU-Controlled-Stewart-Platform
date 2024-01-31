#include "PLL.h"
#include "SysTick.h"

#include "IMU.h"
#include "ICM-20948.h"

static unsigned int time = 0;
static uint16_t gyroXHOut = 0;
static uint16_t gyroXLOut = 0;
static uint16_t gyroYHOut = 0;
static uint16_t gyroYLOut = 0;
static uint16_t whoAmI = 0;

void Delay(timeInMs)
{
  SysTick_Wait10ms(timeInMs / 10);
}

int main(void)
{
  DELAY_FUNC Delay_Func = &Delay;

  // Initialize SysTick
  SysTick_Init();

  // Initialize the PLL
  PLL_Init();

  // Initialize the IMU with the SYS_CLOCK and to communicate at 1MHz
  IMU_Init(80e6, 1e6, Delay_Func);

  whoAmI = IMU_Read(INT_ENABLE_ADDR);

  while (1)
  {
    time = 0;

    IMU_Read(PWR_MGMT_2_ADDR);

    gyroXHOut = IMU_Read(GYRO_XOUT_H_ADDR);
    gyroXLOut = IMU_Read(GYRO_XOUT_L_ADDR);
    gyroYHOut = IMU_Read(GYRO_YOUT_H_ADDR);
    gyroYLOut = IMU_Read(GYRO_YOUT_L_ADDR);

    while (++time < 16e4)
      ;
  }
}
