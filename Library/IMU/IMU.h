#include <stdbool.h>
#include <stdint.h>

#include "UTILS/UTILS.h"
#include "ICM-20948/ICM-20948.h"

/**
 * @brief
 * @param SYS_CLK
 * @param position
 */
void IMU_Init(uint32_t SYS_CLK, volatile Position *position);

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
