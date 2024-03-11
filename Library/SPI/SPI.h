#include <stdint.h>

#include "tm4c123gh6pm.h"

typedef enum {
  SPI_MODE0 = (SSI_CR0_FRF_MOTO & ~(SSI_CR0_SPO | SSI_CR0_SPH)),
  SPI_MODE1 = ((SSI_CR0_FRF_MOTO | SSI_CR0_SPH) & ~SSI_CR0_SPO),
  SPI_MODE2 = ((SSI_CR0_FRF_MOTO | SSI_CR0_SPO) & ~SSI_CR0_SPH),
  SPI_MODE3 = (SSI_CR0_FRF_MOTO | SSI_CR0_SPO | SSI_CR0_SPH),
} SPI_MODE;

typedef enum {
  SPI_DATA_16 = SSI_CR0_DSS_16,
  SPI_DATA_8 = SSI_CR0_DSS_8,
} DATA_SIZE;

void SPI_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, SPI_MODE frameConfig, DATA_SIZE dataSize);

void SPI_Read(uint16_t initData, uint16_t *result, uint32_t length);

void SPI_Write(uint16_t *data, uint32_t length);

void SPI_StartTransmission(void);

void SPI_EndTransmission(void);
