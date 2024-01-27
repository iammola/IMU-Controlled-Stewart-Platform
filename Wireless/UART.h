#include <stdint.h>
#include <stdbool.h>

#define UART_INTERRUPT_PRIORITY 5

void UART_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t RXFIFOLevel, bool useTwoStopBits, bool isEvenParity);

void UART_Transmit(uint8_t *data, uint8_t byteCount);

uint8_t UART_Receive(void);
