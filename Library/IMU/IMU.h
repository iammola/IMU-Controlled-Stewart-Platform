#include <stdbool.h>
#include <stdint.h>

#include "ICM-20948/ICM-20948.h"

extern volatile FusionQuaternion quaternion;
extern volatile bool             HasNewIMUAngles;

/**
 * @brief 
 * @param SYS_CLK 
 */
void IMU_Init(uint32_t SYS_CLK);

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
