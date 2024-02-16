#include <stdint.h>

void SPI2_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, uint8_t frameConfig, uint8_t dataSize);

void SPI2_Read(uint16_t initData, uint16_t *result, uint8_t length);

void SPI2_Write(uint16_t *data, uint8_t length);

void SPI2_StartTransmission(void);

void SPI2_EndTransmission(void);
