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

#define CHANGE_CONTROL_METHOD_LENGTH     1  // Sent contorl method
#define CHANGE_CONTROL_METHOD_ACK_LENGTH 1  // Returned control method
#define NEW_POSITION_LENGTH              16 // (4 bytes per float * 4 quaternion floats)

typedef enum COMMAND {
  CHANGE_CONTROL_METHOD = 0x57,
  CHANGE_CONTROL_METHOD_ACK = 0x58,
  NEW_POSITION = 0x21,
} COMMAND;

typedef struct CommandData {
  struct NewPosition {
    bool    inUse;
    uint8_t data[NEW_POSITION_LENGTH];
  } NewPosition;
  struct ChangeControlMethod {
    bool    inUse;
    uint8_t data[CHANGE_CONTROL_METHOD_LENGTH];
  } ChangeControlMethod;
  struct ChangeControlMethodAck {
    bool    inUse;
    uint8_t data[CHANGE_CONTROL_METHOD_ACK_LENGTH];
  } ChangeControlMethodAck;
} CommandData;

extern CommandData ReceivedCommands;

extern inline void Wireless_Init(uint32_t SYS_CLOCK, bool enableRX);

void Wireless_Transmit(COMMAND cmd, uint8_t *data, uint8_t length);
