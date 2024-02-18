/**
 * Name: Ademola Adedeji
 * Date: 27th January, 2024
 * Course-Code: ELTR-2400
 * Course-Name: Communication Systems II
 *
 * Lab Description: Initializes UART module 4 on Port C pins 4 (Tx) and 5 (Tx).
 *                  Automatically calculates the need for High Speed Mode (HSE),
 *                  and the Integer and Fractional parts of the Baud Rate. It allows
 *                  for configuration of the Data length, Use of Parity and number of
 *                  stop bits. Only uses Receive interrupts, and also allows for configureaiton
 *                  of the FIFO level to trigger the Receive FIFO interrupt.
 */
#include <stdbool.h>
#include <stdint.h>

#define UART_INTERRUPT_PRIORITY 5

#define WORD_8_BIT 3
#define WORD_7_BIT 2
#define WORD_6_BIT 1
#define WORD_5_BIT 0

#define EVEN_PARITY 3
#define ODD_PARITY  1
#define NO_PARITY   0

#define TWO_STOP_BITS true
#define ONE_STOP_BIT false

#define RX_FIFO_7_8 4
#define RX_FIFO_6_8 3
#define RX_FIFO_4_8 2
#define RX_FIFO_2_8 1
#define RX_FIFO_1_8 0

// -------- UART_Enable -------
// Enables the UART, Transmit and Receive operations
// Input: None
// Output: None
void UART_Enable(void);

// -------- UART_Disable -------
// Waits for the UART to be IDLE before clearing the FIFO by disabling it and
// then disables the UART, Transmit and Receive operations
// Input: None
// Output: None
void UART_Disable(void);

void UART_TimeoutInterrupt(void);
void UART_FIFOInterrupt(uint8_t RXFIFOLevel);

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
void UART_Init(uint32_t SYS_CLOCK, uint32_t baudRate, uint8_t wordLength, uint8_t parity, bool useTwoStopBits);

// ----------- UART_Transmit -------------
// Transmits data through the UART line. If the transmit FIFO is full, it blocks further
// processing until there is space to prevent data loss
// Input: data - Data buffer to transmit
//        length - The number of bytes in the data buffer to transmit
// Output: None
void UART_Transmit(uint8_t *data, uint32_t length);

// --------- UART_Receive ------------
// Gets the data received by the UART. It waits for the Receive FIFO to not be empty,
// before returning the data
// Input: data - Location to store received data
//        length - The number of bytes in the data buffer to transmit
// Output: Data received from UART
void UART_Receive(uint8_t *data, uint32_t length);
