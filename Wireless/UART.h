#include <stdint.h>
#include <stdbool.h>

#define UART_INTERRUPT_PRIORITY 5

void UART_Init(uint8_t SYS_CLOCK, bool useHighSpeed, uint32_t baudRate, uint8_t wordLength, uint8_t RXFIFOLevel, bool useTwoStopBits, bool isEvenParity);

void UART_Transmit(uint8_t *data, uint8_t byteCount);

void UART_Receive(uint8_t byteCount, uint8_t *dest);
