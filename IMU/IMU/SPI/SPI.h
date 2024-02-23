#include <stdint.h>

void SPI3_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, uint8_t frameConfig, uint8_t dataSize);

void SPI3_Read(uint16_t initData, uint16_t *result, uint8_t length);

void SPI3_Write(uint16_t *data, uint8_t length);

void SPI3_StartTransmission(void);

void SPI3_EndTransmission(void);
