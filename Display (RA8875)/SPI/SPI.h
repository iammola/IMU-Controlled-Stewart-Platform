#include <stdint.h>

void SPI0_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, uint8_t frameConfig, uint8_t dataSize);

void SPI0_Read(uint16_t initData, uint16_t *result, uint32_t length);

void SPI0_Write(uint16_t *data, uint32_t length);

void SPI0_StartTransmission(void);

void SPI0_EndTransmission(void);
