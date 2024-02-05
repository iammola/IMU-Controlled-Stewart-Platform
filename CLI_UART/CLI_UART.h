#include <stdint.h>
#include <stdbool.h>

#define UART_INTERRUPT_PRIORITY 5

// ----------- UART_Init ------------
// Initializes the UART 4 module in Port C using the specified options
// Input: SYS_CLOCK - System Clock
//        baudRate - The desired baud rate for UART transmission
//        wordLength - The number of bits in the data word
//                     3 for 8 bits, 2 for 7 bits, 1 for 6 bits, and otherwise for 5 bits
//        RXFIFOLevel - The desired level to trigger the Receive FIFO interrupt on
//        parity - If the LSB is 1, it denotes parity is enabled
//                 the 2nd bit being a 1 denotes Even Parity
//        useTwoStopBits - For two stop bits to be used at the end of transmission
// Output: None
void CLI_UART_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t RXFIFOLevel, uint8_t parity, bool useTwoStopBits);

// ----------- UART_Transmit -------------
// Transmits data through the UART line. If the transmit FIFO is full, it blocks further
// processing until there is space to prevent data loss
// Input: data - Data buffer to transmit
//        length - The number of bytes in the data buffer to transmit
// Output: None
void CLI_UART_Transmit(uint8_t *data, uint8_t length);

// --------- UART_Receive ------------
// Gets the data received by the UART. It waits for the Receive FIFO to not be empty,
// before returning the data
// Input: data - Location to store received data
//        length - The number of bytes in the data buffer to transmit
// Output: Data received from UART
void CLI_UART_Receive(uint8_t *data, uint8_t length);
