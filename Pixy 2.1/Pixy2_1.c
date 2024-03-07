#include <stdbool.h>

#include "SPI/SPI.h"
#include "tm4c123gh6pm.h"

#define SPI_SPEED 2e6

#define CHECKSUM_BYTE    0xC1AF
#define NO_CHECKSUM_BYTE ((0xC1 << 0) | (0xAE << 8)) // should be 0xC1AE, but sent as little endian, so LSB first

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

static bool Pixy2_1_Transaction(PACKET_TYPE reqType, PACKET_TYPE resType, uint16_t *txBuff, uint16_t *rxBuff, uint32_t rXLength, uint32_t tXLength);
static void Pixy2_1_GetVersion(void);

void Pixy2_1_Init(uint32_t SYS_CLOCK) {
  SPI1_Init(SYS_CLOCK, SPI_SPEED, SSI_CR0_FRF_MOTO | SSI_CR0_SPO | SSI_CR0_SPH, SSI_CR0_DSS_16);

  Pixy2_1_GetVersion();
}

bool Pixy2_1_Checksum(uint16_t *response);

static bool Pixy2_1_Transaction(PACKET_TYPE reqType, PACKET_TYPE resType, uint16_t *txBuff, uint16_t *rxBuff, uint32_t rXLength, uint32_t tXLength) {
  int32_t checksum = 0;
  uint8_t checksumIdx = 0;
  uint8_t metadataIdx = 0;

  uint16_t request[] = {
      NO_CHECKSUM_BYTE,
      (reqType << 8) | tXLength,
  };

  SPI1_Write(request, 2);
  if (tXLength > 0) {
    SPI1_Write(txBuff, (tXLength + 1) / 2); // Transmit data with 16 byte length rounded up to 1
  }

  // Receive response with length reduced to d-word format, + 5/2 for the metadata bytes and +1/2 for rounding errors
  SPI1_Read(0x00, rxBuff, (rXLength + 5 + 1) / 2);

  // First d-byte must match checksum d-byte. Next d-byte much match response type and length expected
  if (rxBuff[metadataIdx++] != CHECKSUM_BYTE || rxBuff[metadataIdx++] != ((resType << 8) | rXLength))
    return false;

  checksum = (rxBuff[metadataIdx++] << 0) || (rxBuff[metadataIdx++] << 8); // Get total checksum

  for (checksumIdx = 0; checksumIdx < rXLength; checksumIdx++) {
    checksum -= rxBuff[metadataIdx + checksumIdx]; // Remove byte from total data
  }

  // Checksum check failed
  if (checksum != 0)
    return false;

  return true;
}

void Pixy2_1_GetVersion(void) {
  uint16_t request[] = {
      NO_CHECKSUM_BYTE,
      (VERSION_REQUEST << 8) | (0 /* No data sent */),
  };
  uint16_t response[8] = {0};

  bool valid = Pixy2_1_Transaction(VERSION_REQUEST, VERSION_RESPONSE, (void *)0, response, 8, 0); // Expecting 8 double-bytes after metadata
  if (!valid)
    return;

  if (((response[0]) != 0x2200) || // Hardware version
      ((response[1]) != 0x0003) || // Firmware version number
      ((response[2]) != 0x000a) || // Firmware build number
      ((response[3]) != 0x6567) || // Byte 0 & 1 of Firmware type ASCII string
      ((response[4]) != 0x656e) || // Byte 2 & 3 of Firmware type ASCII string
      ((response[5]) != 0x6172) || // Byte 4 & 5 of Firmware type ASCII string
      ((response[6]) != 0x006c) || // Byte 6 & 7 of Firmware type ASCII string
      ((response[7]) != 0x0000)    // Byte 8 & 9 of Firmware type ASCII string
  ) {
    while (1) { // Invalid Response
    }
  }
}

void Pixy2_1_SetLED(uint8_t R, uint8_t G, uint8_t B) {
  uint16_t colorPayload[] = {(R << 16) | (G << 8), (B << 8)};
  uint16_t response[4] = {0};

  bool valid = Pixy2_1_Transaction(LED_REQUEST, RESULT_RESPONSE, colorPayload, response, 4, 3);
  if (!valid)
    return;

  // Some check here to confirm it being changed
}
