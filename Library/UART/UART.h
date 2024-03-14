/**
 * @file UART.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdbool.h>
#include <stdint.h>

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
