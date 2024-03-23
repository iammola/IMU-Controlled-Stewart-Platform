/**
 * @file Wireless.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stdint.h>

#include "HC-12/HC-12.h"

#define DATA_OFFSET 2

typedef enum COMMAND {
  CHANGE_CONTROL_METHOD = 0x57,
  NEW_QUATERNION = 0x21,
} COMMAND;

static inline void Wireless_Init(uint32_t SYS_CLOCK) {
  HC12_Init();
  HC12_Config(SYS_CLOCK, BAUD_115200, TX_20dBm); // Use 115200 bps, 20 dBm
}

static inline void Wireless_Transmit(uint8_t *data, uint8_t length) {
  HC12_SendData(data, length);
}
