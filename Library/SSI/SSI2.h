#include <stdint.h>

#include "SSI.h"

void SSI2_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, SSI_MODE frameConfig, DATA_SIZE dataSize);

void SSI2_Read(uint16_t initData, uint16_t *result, uint8_t length);

void SSI2_Write(uint16_t *data, uint8_t length);

void SSI2_StartTransmission(void);

void SSI2_EndTransmission(void);
