#include <stdint.h>
#include <stdbool.h>

void UART_Init(uint8_t SYS_CLOCK, bool useHighSpeed, uint32_t baudRate, uint8_t wordLength, uint8_t RXFIFOLevel, bool useTwoStopBits, bool isEvenParity);

void UART_TransmitByte(uint32_t data, uint8_t byteCount);

uint32_t UART_ReceiveByte(uint8_t byteCount);
