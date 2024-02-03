#include <stdint.h>

#include "PLL.h"
#include "SysTick.h"

#include "ICM-20948.h"

static unsigned int time = 0;
static uint16_t gyroXOut = 0;
static uint16_t gyroYOut = 0;

static uint8_t gyroXHOut = 0;
static uint8_t gyroXLOut = 0;
static uint8_t gyroYHOut = 0;
static uint8_t gyroYLOut = 0;

static uint8_t accelYOut = 0;
static uint8_t accelXOut = 0;

static uint8_t accelXHOut = 0;
static uint8_t accelXLOut = 0;
static uint8_t accelYHOut = 0;
static uint8_t accelYLOut = 0;

static uint8_t whoAmI = 0;
static uint8_t userCtrl = 0;
static uint8_t pwrMgmt1 = 0;
static uint8_t pwrMgmt2 = 0;
static uint8_t intEnable = 0;
static uint8_t intPinCfg = 0;

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

  IMU_Read(WHO_AM_I_ADDR, &whoAmI);
  IMU_Read(PWR_MGMT_1_ADDR, &pwrMgmt1);
  IMU_Read(PWR_MGMT_2_ADDR, &pwrMgmt2);
  IMU_Read(USER_CTRL_ADDR, &userCtrl);
  IMU_Read(INT_ENABLE_ADDR, &intEnable);
  IMU_Read(INT_PIN_CFG_ADDR, &intPinCfg);

  while (1)
  {
    IMU_Read(GYRO_XOUT_H_ADDR, &gyroXHOut);
    IMU_Read(GYRO_XOUT_L_ADDR, &gyroXLOut);
    gyroXOut = ((gyroYHOut << 8) | gyroYLOut);

    IMU_Read(GYRO_YOUT_H_ADDR, &gyroYHOut);
    IMU_Read(GYRO_YOUT_L_ADDR, &gyroYLOut);
    gyroYOut = ((gyroYHOut << 8) | gyroYLOut);

    IMU_Read(ACCEL_XOUT_H_ADDR, &accelXHOut);
    IMU_Read(ACCEL_XOUT_L_ADDR, &accelXLOut);
    accelXOut = ((accelXHOut << 8) | accelXLOut);

    IMU_Read(ACCEL_YOUT_H_ADDR, &accelYHOut);
    IMU_Read(ACCEL_YOUT_L_ADDR, &accelYLOut);
    accelYOut = ((accelYHOut << 8) | accelYLOut);
  }
}
