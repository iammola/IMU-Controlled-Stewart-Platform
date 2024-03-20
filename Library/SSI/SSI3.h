#include <stdint.h>

#include "SSI.h"

#define SSI3_CLK_BIT (1 << 0) // (PD0) SSI3CLK
#define SSI3_FSS_BIT (1 << 1) // (PD1) SSI3FSS (Chip Select)
#define SSI3_RX_BIT  (1 << 2) // (PD2) SSI3Rx
#define SSI3_TX_BIT  (1 << 3) // (PD3) SSI3Tx

#define SSI3_PINS      (unsigned)(SSI3_CLK_BIT | SSI3_FSS_BIT | SSI3_RX_BIT | SSI3_TX_BIT)
#define SSI3_PCTL      (unsigned)(GPIO_PCTL_PD0_SSI3CLK | GPIO_PCTL_PD1_SSI3FSS | GPIO_PCTL_PD2_SSI3RX | GPIO_PCTL_PD3_SSI3TX)
#define SSI3_PCTL_MASK (uint32_t)(GPIO_PCTL_PD0_M | GPIO_PCTL_PD1_M | GPIO_PCTL_PD2_M | GPIO_PCTL_PD3_M)

void SSI3_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, SSI_MODE frameConfig, DATA_SIZE dataSize);

void SSI3_Read(uint16_t initData, uint16_t *result, uint32_t length);

void SSI3_Write(uint16_t *data, uint32_t length);

void SSI3_StartTransmission(void);

void SSI3_EndTransmission(void);
