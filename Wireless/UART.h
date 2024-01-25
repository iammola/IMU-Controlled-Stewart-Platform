#include <stdint.h>
#include <stdbool.h>

void UART_Init(uint8_t SYS_CLOCK, bool useHighSpeed, uint32_t baudRate, uint8_t wordLength, bool useTwoStopBits, bool isEvenParity, bool enableFIFO);

void UART_Receive(void);

void UART_Transmit(void);
