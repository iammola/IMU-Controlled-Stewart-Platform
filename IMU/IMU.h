#include "stdint.h"

typedef struct Coords_STRUCT {
  int16_t x;
  int16_t y;
} Coords;

void IMU_GetMagReadings(Coords *dest);
void IMU_GetGyroReadings(Coords *dest);
void IMU_GetAccelReadings(Coords *dest);
void IMU_Init(uint32_t SYS_CLK, uint32_t SSI_CLK);
