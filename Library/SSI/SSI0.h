#include <stdint.h>

#include "SSI.h"

void SSI0_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, SSI_MODE frameConfig, DATA_SIZE dataSize);

void SSI0_Read(uint16_t initData, uint16_t *result, uint32_t length);

void SSI0_Write(uint16_t *data, uint32_t length);

void SSI0_StartTransmission(void);

void SSI0_EndTransmission(void);
