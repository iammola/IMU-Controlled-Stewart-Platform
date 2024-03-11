#include <stdint.h>

#include "SSI.h"

void SSI3_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, SSI_MODE frameConfig, DATA_SIZE dataSize);

void SSI3_Read(uint16_t initData, uint16_t *result, uint8_t length);

void SSI3_Write(uint16_t *data, uint8_t length);

void SSI3_StartTransmission(void);

void SSI3_EndTransmission(void);
