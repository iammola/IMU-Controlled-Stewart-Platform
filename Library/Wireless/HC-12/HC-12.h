#include <stdbool.h>
#include <stdint.h>

// TRIGGER ON 12/16 bytes full
#define METADATA_SIZE 2
#define MAX_MESSAGE_SIZE 32

#define SYNC_WORD 0xEA

typedef enum {
  TX_20dBm = 8,
  TX_17dBm = 7,
  TX_14dBm = 6,
  TX_11dBm = 5,
  TX_8dBm = 4,
  TX_5dBm = 3,
  TX_2dBm = 2,
  TX_N_1dBm = 1,
} TX_POWER;

typedef enum {
  BAUD_1200 = 1200,
  BAUD_2400 = 2400,
  BAUD_4800 = 4800,
  BAUD_9600 = 9600,
  BAUD_19200 = 19200,
  BAUD_38400 = 38400,
  BAUD_57600 = 57600,
  BAUD_115200 = 115200,
} BAUD_RATE;

/**
 * @brief Initializes the HC-12 VCC and SET pins. Required to call `HC12_Config` to setup
 * the UART pins at the desired Baud Rate
 * @param
 */
void HC12_Init(void);

/**
 * @brief Configures the HC-12 module to use the desired Baud Rate and TX Power Level.
 * Verifies the Firmware version matches the expectation, using Channel 1 (433 MHz) and
 * FU3 transmission mode
 * @param SYS_CLOCK System clock speed
 * @param baud Desired baud rate for communication, changes the Over-the-Air baud rate
 * @param powerLevel Desired power level for transmission
 * @param enableRX Enable RX FIFO
 */
void HC12_Config(uint32_t SYS_CLOCK, BAUD_RATE baud, TX_POWER powerLevel, bool enableRX);

/**
 * @brief Used to send data at the configured baud rate through the air
 * @param data
 * @param length
 * @return
 */
bool HC12_SendData(uint8_t *data, uint8_t length);

/**
 * @brief Parses the received data into its appropriate RX buffers
 * @param RX_Data_Buffer 
 */
extern inline void HC12_ReceiveHandler(uint8_t *RX_Data_Buffer);
