#define SSI_MODULE 1

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>  // Using snprintf
#include <string.h> // Using memcpy

#include "SSI/SSI3.h"
#include "tm4c123gh6pm.h"

#include "Pixy2_1.h"

#define SSI_SPEED 175e4 // 1.75 MHz

#define RX_BUF_SIZE 100

#define CHECKSUM_SYNC_BYTE    0xAFC1 // should be 0xC1AF
#define NO_CHECKSUM_SYNC_BYTE 0xAEC1 // should be 0xC1AE, but sent as little endian, so LSB first

typedef enum PACKET_TYPE {
  /*  */
  VERSION_REQUEST = 0x0e,
  VERSION_RESPONSE = 0x0f,
  /*  */
  CHANGE_PROG_REQUEST = 0x02,
  BRIGHTNESS_REQUEST = 0x10,
  SERVO_REQUEST = 0x12,
  LED_REQUEST = 0x14,
  LAMP_REQUEST = 0x16,
  FPS_REQUEST = 0x18,
  RESULT_RESPONSE = 0x01,
  ERROR_RESPONSE = 0x03,
  /*  */
  RESOLUTION_REQUEST = 0x0c,
  RESOLUTION_RESPONSE = 0x0d,
} PACKET_TYPE;

static bool Pixy2_1_Transaction(PACKET_TYPE reqType, PACKET_TYPE resType, uint16_t *txBuff, uint16_t *rxBuff, uint32_t rxLength, uint32_t txLength);
static void Pixy2_1_GetVersion(void);

void Pixy2_1_Init(uint32_t SYS_CLOCK) {
  // Arduino SPI Mode 1
  SSI3_Init(SYS_CLOCK, SSI_SPEED, SSI_MODE1, SSI_DATA_16);

  Pixy2_1_GetVersion();
}

static bool Pixy2_1_Transaction(PACKET_TYPE reqType, PACKET_TYPE resType, uint16_t *txBuff, uint16_t *rxBuff, uint32_t rxLength, uint32_t txLength) {
  uint8_t rxChecksumDataIdx = 0;
  int32_t rxChecksum = 0;

  int16_t syncIdx = -1;
  uint8_t loopIdx = 10;

  uint16_t totalRxBuff[RX_BUF_SIZE] = {0};

  uint16_t request[] = {NO_CHECKSUM_SYNC_BYTE, (uint16_t)(((unsigned)reqType << 8) | txLength)};

  SSI3_StartTransmission();

  SSI3_Write(request, 2);
  if (txLength > 0) {
    SSI3_Write(txBuff, (txLength + 1) / 2); // Transmit data with 16 byte length rounded up to 1
  }

  if (rxLength < 1) {
    SSI3_EndTransmission(); // No data to send, so end transmission early
    return true;
  }

  // Receive response with length reduced to d-word format, + 6/2 for the metadata bytes and +1/2 for rounding errors
  SSI3_Read(0x00, totalRxBuff, (rxLength + 6 + 1 + 8) / 2);

  SSI3_EndTransmission();

  // Attempt to handle random unusable bytes before actual data
  for (loopIdx = 0; loopIdx < RX_BUF_SIZE; loopIdx++) {
    if (totalRxBuff[loopIdx] == CHECKSUM_SYNC_BYTE) {
      syncIdx = loopIdx;
      break;
    }
  }

  if (syncIdx < 0 ||                                                    // The idx of the checksum sync byte must exist
      totalRxBuff[syncIdx + 0] != CHECKSUM_SYNC_BYTE ||                 // First d-byte must match checksum d-byte. (Redundant following prev)
      totalRxBuff[syncIdx + 1] != (((unsigned)resType << 8) | rxLength) // Next d-byte much match response type and length expected
  ) {
    return false;
  }

  // Get total checksum
  rxChecksum = ((totalRxBuff[syncIdx + 2] & 0xFF) << 8) | ((totalRxBuff[syncIdx + 2] & 0xFF00) >> 8);

  for (loopIdx = 0; loopIdx < ((rxLength + 1) / 2); loopIdx++) {
    rxChecksum -= ((totalRxBuff[syncIdx + 3 + loopIdx] >> 8) & 0xFF); // Remove byte from total checksum
    rxChecksum -= (totalRxBuff[syncIdx + 3 + loopIdx] & 0xFF);        // Remove byte from total checksum
  }

  // Checksum check failed
  if (rxChecksum != 0)
    return false;

  // A char of a string is one byte, so since copying to a 16 bit array, there's 2 taken for each element
  // The calculation ((len + 1) / 2) * 2) seems redundant, but it's my way to have the least-significant byte right-shifted
  // by 8 bits for the most-significant-byte
  // E.g. Len = 18, Size = ((18 + 1)/2) * 2 = (19/2)*2 = 9.5*2 = 9*2 = 18 bytes.
  // E.g. Len = 19, Size = ((19 + 1)/2) * 2 = (20/2)*2 = 10*2 = 20 bytes.
  memcpy(rxBuff, totalRxBuff + syncIdx + 4, ((rxLength + 1) / 2) * 2);

  return true;
}

void Pixy2_1_GetVersion(void) {
  uint16_t response[8] = {0};

  char versionString[100] = "";

  bool valid = Pixy2_1_Transaction(VERSION_REQUEST, VERSION_RESPONSE, 0, response, 16, 0); // Expecting 8 double-bytes after metadata
  if (!valid) {
    while (1) {
    }
  }

  snprintf(versionString, 100, "Hardware Version: %d%d", (response[0] & 0xFF00) >> 8, response[0] & 0xFF);  // Hardware version
  snprintf(versionString, 100, "Firmware Version: %d.%d", (response[1] & 0xFF00) >> 8, response[1] & 0xFF); // Firmware version number
  snprintf(versionString, 100, "Firmware Build: %d%d", (response[2] & 0xFF00) >> 8, response[2] & 0xFF);    // Firmware build number
  snprintf(versionString, 100, "Firmware Type: %c%c%c%c%c%c%c%c%c%c",                                       //
           (response[3] & 0xFF00) >> 8, response[3] & 0xFF, // Byte 0 & 1 of Firmware type ASCII string
           (response[4] & 0xFF00) >> 8, response[4] & 0xFF, // Byte 2 & 3 of Firmware type ASCII string
           (response[5] & 0xFF00) >> 8, response[5] & 0xFF, // Byte 4 & 5 of Firmware type ASCII string
           (response[6] & 0xFF00) >> 8, response[6] & 0xFF, // Byte 6 & 7 of Firmware type ASCII string
           (response[7] & 0xFF00) >> 8, response[7] & 0xFF  // Byte 8 & 9 of Firmware type ASCII string
  );
}

void Pixy2_1_SetLED(uint8_t R, uint8_t G, uint8_t B) {
  uint16_t colorPayload[] = {(uint16_t)((R << 16) | (G << 8)), (uint16_t)(B << 8)};
  uint16_t response[2] = {0};

  bool valid = Pixy2_1_Transaction(LED_REQUEST, RESULT_RESPONSE, colorPayload, response, 4, 3);
  if (!valid)
    return;

  // Some check here to confirm it being changed
}
