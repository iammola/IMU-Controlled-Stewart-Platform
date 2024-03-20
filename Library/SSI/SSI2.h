#include <stdint.h>

#include "SSI.h"

#define SSI2_CLK_BIT (1 << 4) // (PB4) SSI2CLK
#define SSI2_FSS_BIT (1 << 5) // (PB5) SSI2FSS (Chip Select)
#define SSI2_RX_BIT  (1 << 6) // (PB6) SSI2Rx
#define SSI2_TX_BIT  (1 << 7) // (PB7) SSI2Tx

#define SSI2_PINS      (unsigned)(SSI2_CLK_BIT | SSI2_FSS_BIT | SSI2_RX_BIT | SSI2_TX_BIT)
#define SSI2_PCTL      (unsigned)(GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB5_SSI2FSS | GPIO_PCTL_PB6_SSI2RX | GPIO_PCTL_PB7_SSI2TX)
#define SSI2_PCTL_MASK (uint32_t)(GPIO_PCTL_PB4_M | GPIO_PCTL_PB5_M | GPIO_PCTL_PB6_M | GPIO_PCTL_PB7_M)

void SSI2_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, SSI_MODE frameConfig, DATA_SIZE dataSize);

void SSI2_Read(uint16_t initData, uint16_t *result, uint32_t length);

void SSI2_Write(uint16_t *data, uint32_t length);

void SSI2_StartTransmission(void);

void SSI2_EndTransmission(void);
