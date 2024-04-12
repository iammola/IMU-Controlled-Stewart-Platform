#include <stdbool.h>
#include <stdint.h>

#include "UTILS/UTILS.h"

/**
 * @brief
 * @param SYS_CLK
 * @param SAMPLE_RATE
 * @param position
 */
void IMU_Init(uint32_t SYS_CLK, uint32_t SAMPLE_RATE, volatile Position *position);

/**
 * @brief
 * @param initializeFusion
 */
void IMU_Enable(bool initializeFusion);

/**
 * @brief
 * @param
 */
void IMU_Disable(void);
