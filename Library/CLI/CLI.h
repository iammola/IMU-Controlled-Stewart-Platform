#include <stdbool.h>
#include <stdint.h>

#define CLI_TXT_BUF 1000

#define WORD_8_BIT 3
#define WORD_7_BIT 2
#define WORD_6_BIT 1
#define WORD_5_BIT 0

#define EVEN_PARITY 3
#define ODD_PARITY  1
#define NO_PARITY   0

#define TWO_STOP_BITS true
#define ONE_STOP_BIT  false

#define RX_FIFO_OFF 5
#define RX_FIFO_7_8 4
#define RX_FIFO_6_8 3
#define RX_FIFO_4_8 2
#define RX_FIFO_2_8 1
#define RX_FIFO_1_8 0

#define UART_INTERRUPT_PRIORITY 1

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
void CLI_Init(const uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t RXFIFOLevel, uint8_t parity, bool useTwoStopBits);

// ----------- UART_Transmit -------------
// Transmits data through the UART line. If the transmit FIFO is full, it blocks further
// processing until there is space to prevent data loss
// Input: data - String to transmit
// Output: None
void CLI_Write(char *data);

// --------- UART_Receive ------------
// Gets the data received by the UART. It waits for the Receive FIFO to not be empty,
// before returning the data
// Input: None
// Output: Data received from UART
uint8_t CLI_Read(void);

/* Enables the UART module, TX and RX operations */
void CLI_Enable(void);

/* Disables the UART module */
void CLI_Disable(void);
