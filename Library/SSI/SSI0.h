#include <stdint.h>

#include "SSI.h"

#define SSI0_CLK_BIT (1 << 2) // (PA2) SSI0CLK
#define SSI0_FSS_BIT (1 << 3) // (PA3) SSI0FSS (Chip Select)
#define SSI0_RX_BIT  (1 << 4) // (PA4) SSI0Rx
#define SSI0_TX_BIT  (1 << 5) // (PA5) SSI0Tx

#define SSI0_PINS      (unsigned)(SSI0_CLK_BIT | SSI0_FSS_BIT | SSI0_RX_BIT | SSI0_TX_BIT)
#define SSI0_PCTL      (unsigned)(GPIO_PCTL_PA2_SSI0CLK | GPIO_PCTL_PA3_SSI0FSS | GPIO_PCTL_PA4_SSI0RX | GPIO_PCTL_PA5_SSI0TX)
#define SSI0_PCTL_MASK (uint32_t)(GPIO_PCTL_PA2_M | GPIO_PCTL_PA3_M | GPIO_PCTL_PA4_M | GPIO_PCTL_PA5_M)

void SSI0_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, SSI_MODE frameConfig, DATA_SIZE dataSize);

void SSI0_Read(uint16_t initData, uint16_t *result, uint32_t length);

void SSI0_Write(uint16_t *data, uint32_t length);

void SSI0_StartTransmission(void);

void SSI0_EndTransmission(void);
