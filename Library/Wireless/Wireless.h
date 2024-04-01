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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "HC-12/HC-12.h"

#define DATA_OFFSET 2

#define CHANGE_CONTROL_METHOD_LENGTH 1
#define NEW_POSITION_LENGTH          28 // (4 bytes per flaot * (4 quaternion floats) + (3 translation floats))

typedef enum COMMAND {
  CHANGE_CONTROL_METHOD = 0x57,
  CHANGE_CONTROL_METHOD_ACK = 0x58,
  NEW_POSITION = 0x21,
  PING = 0x05,
} COMMAND;

inline void Wireless_Init(uint32_t SYS_CLOCK, bool enableRX) {
  HC12_Init();
  HC12_Config(SYS_CLOCK, BAUD_115200, TX_20dBm, enableRX); // Use 115200 bps, 20 dBm
}

inline void Wireless_Transmit(COMMAND cmd, uint8_t *data, uint8_t length) {
  TX_Data_Buffer[0] = cmd;
  TX_Data_Buffer[1] = length;
  memcpy(TX_Data_Buffer + 2, data, length);

  HC12_SendData(TX_Data_Buffer, length + 2);
}
