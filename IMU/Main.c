#include <stdint.h>

#include "PLL.h"
#include "SysTick.h"

#include "IMU.h"
#include "ICM-20948.h"

static unsigned int time = 0;
static uint8_t gyroXHOut = 0;
static uint8_t gyroXLOut = 0;
static uint8_t gyroYHOut = 0;
static uint8_t gyroYLOut = 0;

static uint8_t accelXHOut = 0;
static uint8_t accelXLOut = 0;
static uint8_t accelYHOut = 0;
static uint8_t accelYLOut = 0;

static uint8_t whoAmI = 0;

static uint8_t pwrMgmt1 = 0;
static uint8_t pwrMgmt2 = 0;

void Delay(uint32_t timeInMs);

void Delay(uint32_t timeInMs)
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

  // IMU_Read(INT_ENABLE_ADDR);

  IMU_Read(WHO_AM_I_ADDR, &whoAmI);

  IMU_Read(PWR_MGMT_1_ADDR, &pwrMgmt1);
  IMU_Read(PWR_MGMT_2_ADDR, &pwrMgmt2);

  while (1)
  {
    // IMU_Read(GYRO_XOUT_H_ADDR, &gyroXHOut);
    // IMU_Read(GYRO_XOUT_L_ADDR, &gyroXLOut);
    // IMU_Read(GYRO_YOUT_H_ADDR, &gyroYHOut);
    // IMU_Read(GYRO_YOUT_L_ADDR, &gyroYLOut);

    IMU_Read(ACCEL_XOUT_H_ADDR, &accelXHOut);
    IMU_Read(ACCEL_XOUT_L_ADDR, &accelXLOut);
    IMU_Read(ACCEL_YOUT_H_ADDR, &accelYHOut);
    IMU_Read(ACCEL_YOUT_L_ADDR, &accelYLOut);

    time = 0;
    while (++time < 1e3)
      ;
  }
}
