#include <stdint.h>

#include "tm4c123gh6pm.h"

typedef enum {
  SPI_MODE0 = (SSI_CR0_FRF_MOTO & ~(SSI_CR0_SPO | SSI_CR0_SPH)),
  SPI_MODE1 = ((SSI_CR0_FRF_MOTO | SSI_CR0_SPO) & ~SSI_CR0_SPH),
  SPI_MODE2 = ((SSI_CR0_FRF_MOTO | SSI_CR0_SPH) & ~SSI_CR0_SPO),
  SPI_MODE3 = (SSI_CR0_FRF_MOTO | SSI_CR0_SPO | SSI_CR0_SPH),
} SPI_MODE;

typedef enum {
  SPI_DATA_16 = SSI_CR0_DSS_16,
  SPI_DATA_8 = SSI_CR0_DSS_8,
} DATA_SIZE;

void SPI3_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, SPI_MODE frameConfig, DATA_SIZE dataSize);

void SPI3_Read(uint16_t initData, uint16_t *result, uint8_t length);

void SPI3_Write(uint16_t *data, uint8_t length);

void SPI3_StartTransmission(void);

void SPI3_EndTransmission(void);
