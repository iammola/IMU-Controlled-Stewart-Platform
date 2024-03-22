#include <stdbool.h>
#include <stdint.h>

#include "ICM-20948/ICM-20948.h"
#include "Quaternion/Quaternion.h"

/**
 * @brief
 * @param SYS_CLK
 * @param quatDest
 * @param hasNewData
 */
void IMU_Init(uint32_t SYS_CLK, volatile Quaternion *quatDest, volatile bool *hasNewData);

/**
 * @brief
 * @param
 */
void IMU_Enable(void);

/**
 * @brief
 * @param
 */
void IMU_Disable(void);
