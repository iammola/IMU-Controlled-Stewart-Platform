#include <stdbool.h>
#include <stdint.h>

typedef void (*RX_Data_Handler)(void);

extern bool    HasNewData;
extern uint8_t RX_Data_Buffer[7];

void HC12_Init(uint32_t SYS_CLOCK, uint32_t BAUD, RX_Data_Handler OnRX);
bool HC12_SendData(uint8_t *data, uint8_t length);
