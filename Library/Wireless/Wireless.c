/**
 * @file Wireless.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "HC-12/HC-12.h"
#include "Wireless.h"

#define BUFFER_COUNT 5

typedef struct Buffer {
  bool    inUse;
  uint8_t data[MAX_MESSAGE_SIZE];
} Buffer;

static Buffer *txBufferPtr;
static uint8_t active_buffer = 0x00;
static Buffer  TX_Data_Buffer[BUFFER_COUNT] = {0};

CommandData ReceivedCommands = {0};

/**
 * @brief Initializes and configures the HC-12 module with constant parameters
 * @param SYS_CLOCK System Clock speed
 * @param enableRX Allow incoming transmissions
 */
void Wireless_Init(const uint32_t SYS_CLOCK, bool enableRX) {
  HC12_Init();
  HC12_Config(SYS_CLOCK, BAUD_115200, TX_20dBm, enableRX); // Use 115200 bps, 20 dBm
}

inline void HC12_ReceiveHandler(uint8_t *RX_Data_Buffer) {
  struct {
    bool    inUse;
    bool    isNew;
    uint8_t data[MAX_MESSAGE_SIZE];
  } *rxBufferPtr; // Has to match order of each property of the CommandData struct.

  uint8_t dataSize = 0x00;
  COMMAND cmd = RX_Data_Buffer[0];

  switch (cmd) {
    case CHANGE_CONTROL_METHOD:
      rxBufferPtr = &ReceivedCommands.ChangeControlMethod;
      dataSize = sizeof(ReceivedCommands.ChangeControlMethod);
      break;
    case CHANGE_CONTROL_METHOD_ACK:
      rxBufferPtr = &ReceivedCommands.ChangeControlMethodAck;
      dataSize = sizeof(ReceivedCommands.ChangeControlMethodAck);
      break;
    case NEW_POSITION:
      rxBufferPtr = &ReceivedCommands.NewPosition;
      dataSize = sizeof(ReceivedCommands.NewPosition);
      break;
  }

  if (rxBufferPtr != NULL && !rxBufferPtr->inUse) { // Only save new data if not currently in use
    rxBufferPtr->isNew = true;
    memcpy(rxBufferPtr->data, RX_Data_Buffer + 1, dataSize);
  }
}

/**
 * @brief Packages the data to be transmitted into one of the available buffers.
 * @param cmd COMMAND for data transfer
 * @param data Data applicable to command
 * @param length Number of bytes in data to send
 */
void Wireless_Transmit(COMMAND cmd, uint8_t *data, uint8_t length) {
  do {
    txBufferPtr = &TX_Data_Buffer[active_buffer]; // Get pointer to buffer

    if (txBufferPtr->inUse)
      active_buffer = (active_buffer + 1) % BUFFER_COUNT; // Increment buffer idx
    else
      break;
  } while (1); // Check if buffer is in use, else get next buffer pointer

  txBufferPtr->inUse = true; // Set buffer flag

  txBufferPtr->data[0] = cmd;                  // Send command
  memcpy(txBufferPtr->data + 1, data, length); // Copy data

  HC12_SendData(txBufferPtr->data, length + 1); // Send data

  txBufferPtr->inUse = false; // Allow buffer to be used
}
