#include <stdint.h>
#include <stdbool.h>

#define MODULE_FREQUENCY 915e6      // 915 MHz (Marked on board) (890MHz - 1020 MHz)
#define F_XOSC 32e6                 // 32MHz (Table 2.3.2)
#define F_STEP (F_XOSC / (1 << 19)) // (Section 3.2.3.3)

typedef enum ADDRESSES {
  /* DATA REGISTER */
  FIFO = 0x00,
  /* CONFIGURATION REGISTERS */
  OPERATION_MODE = 0x01,
  DATA_MODULATION = 0x02,
  BITRATE_FIRST_BYTE = 0x03,
  BITRATE_LAST_BYTE = 0x04,
  DEVIATION_FIRST_BYTE = 0x05,
  DEVIATION_LAST_BYTE = 0x06,
  CARRIER_FREQUENCY_FIRST_BYTE = 0x07,
  CARRIER_FREQUENCY_MID_BYTE = 0x08,
  CARRIER_FREQUENCY_LAST_BYTE = 0x09,
  LISTEN_1 = 0x0D,
  LISTEN_2 = 0x0E,
  LISTEN_3 = 0x0F,
  VERSION = 0x10,
  /* TRANSMITTER REGISTERS */
  PA_LEVEL = 0x11,
  PA_RAMP_TIME = 0x12,
  CURRENT_PROTECTION = 0X13,
  /* RECEIVER REGISTERS */
  RX_BANDWIDTH = 0x19,
  RX_AFC_BANDWIDTH = 0x1A,
  RSSI_CONFIG = 0x23,
  RSSI_VALUE = 0x24,
  /* IRQ REGISTERS */
  DIO_MAPPING_1 = 0x25,
  DIO_MAPPING_2 = 0x26,
  IRQ_FLAGS_1 = 0x27,
  IRQ_FLAGS_2 = 0x28,
  RSSI_THRESHOLD = 0x29,
  TIMEOUT_RX_START = 0x2A,
  TIMEOUT_RSSI_THRESHOLD = 0x2B,
  /* PACKET ENGINE REGISTERS */
  PREAMBLE_FIRST_BYTE = 0x2C,
  PREAMBLE_LAST_BYTE = 0x2D,
  SYNC_CONFIG = 0x2E,
  SYNC_VALUE_1 = 0x2F,
  SYNC_VALUE_2 = 0x30,
  SYNC_VALUE_3 = 0x31,
  PACKET_CONFIG_1 = 0x37,
  PAYLOAD_LENGTH = 0x38,
  PACKET_NODE_ADDR = 0x39,
  AUTO_MODES = 0x3B,
  FIFO_THRESHOLD = 0x3C,
  PACKET_CONFIG_2 = 0x3D,
  AES_KEY_FIRST = 0x3E,
  AES_KEY_LAST = 0x4D,
  /* TEST REGISTERS */
  TEST_DAGC = 0x6F
} ADDRESS;

// OPERATION_MODE
typedef enum MODES {
  OPERATION_MODE_SLEEP = 0x00,
  OPERATION_MODE_STANDBY = 0x04,
  OPERATION_MODE_FS = 0x08,
  OPERATION_MODE_TX = 0x0C,
  OPERATION_MODE_RX = 0x10,
} MODE;
#define OPERATION_MODE_M (unsigned)(7 << 2) // Mask
#define OPERATION_LISTEN_ABORT (unsigned)(1 << 5)
#define OPERATION_LISTEN_ON (unsigned)(1 << 6)
#define OPERATION_SEQUENCER_OFF (unsigned)(1 << 7)

// DATA_MODULATION
#define DATA_MODULATION_GAUSSIAN_1 (unsigned)(1 << 0) // (Bits 1:0)
#define DATA_MODULATION_FSK (unsigned)0x00        // (Bits 4:3)
#define DATA_MODULATION_MODE (unsigned)0x00       // (Bits 6:5)

// LISTEN_1
#define LISTEN_END_IDLE_RESUME (unsigned)(2 << 1)            // (Bits 2:1)
#define LISTEN_CRITERIA_THRESHOLD_RSSI (unsigned)(0 << 3)    // (Bit 3) default
#define LISTEN_CRITERIA_THRESHOLD_ADDRESS (unsigned)(1 << 3) // (Bit 3)
#define LISTEN_RESO_IRX_DEFAULT (unsigned)(1 << 4)           // (Bits 5:4) default
#define LISTEN_RESO_IDLE_64u (unsigned)(1 << 6)              // (Bits 7:6)

// RSSI_CONFIG
#define RSSI_CONFIG_START_SAMPLE 0x01
#define RSSI_CONFIG_RESULT_AVAILABLE 0x02

// RSSI_THRESHOLD
#define RSSI_THRESHOLD_DEFAULT (unsigned)0xE4 // Default = 114dBm

// SYNC_CONFIG
#define SYNC_WORD_VERIFICATION (unsigned)0x80
#define SYNC_WORD_BYTE_COUNT_2 (unsigned)(1 << 3) // (Bits 5:3)
#define SYNC_WORD_NO_TOLERANCE (unsigned)0x00     // (Bits 2:0)

// PACKET_CONFIG_1
#define PACKET_VARIABLE_LENGTH (unsigned)0x80
#define PACKET_CRC_ENABLE (unsigned)0x10
#define PACKET_CRC_AUTO_CLEAR_OFF (unsigned)0x08  // (Bit 3)
#define PACKET_ADDRESS_FILTER_NODE (unsigned)0x02 // (Bits 2:1)
#define PACKET_DC_FREE_WHITENING (unsigned)(2 << 5) // (Bits 6:5)

// PAYLOAD_LENGTH
#define PAYLOAD_LENGTH_64 (unsigned)0x40

// FIFO_THRESHOLD
#define FIFO_TX_ON_NOT_EMPTY (unsigned)0x80
#define FIFO_THRESHOLD_LEVEL_DEFAULT (unsigned)(15 << 0) // (Bits 6:0)

// PACKET_CONFIG_2
#define PACKET_AES_ENCRYPTION (unsigned)(1 << 0)  // (Bit 0)
#define PACKET_AUTO_RX_RESTART (unsigned)(1 << 1) // (Bit 1)
#define PACKET_RX_RESTART (unsigned)(1 << 2)      // (Bit 2)
#define PACKET_INTER_RX_DELAY_S 4                 // Shift amount for bits 7:4
#define PACKET_INTER_RX_DELAY_NONE (unsigned)(12 << PACKET_INTER_RX_DELAY_S)

// PA_LEVEL
#define PA0_ON (unsigned)0x80
#define PA1_ON (unsigned)0x40
#define PA2_ON (unsigned)0x20
#define PA_MAX_POWER (unsigned)0x1F

// CURRENT_PROTECTION
#define CURRENT_PROTECTION_ON (unsigned)0x1A

// PA_RAMP_TIME
#define PA_FSK_RAMP_TIME_M (unsigned)0x0F    // Mask
#define PA_FSK_RAMP_TIME_1m (unsigned)0x02   // For 1ms
#define PA_FSK_RAMP_TIME_125u (unsigned)0x05 // For 125us
#define PA_FSK_RAMP_TIME_40u (unsigned)0x09  // Default
#define PA_FSK_RAMP_TIME_20u (unsigned)0x0C  // For 20us

// RX_BANDWIDTH
#define RECEIVER_DC_OFFSET_CUTOFF_FREQ (unsigned)(7 << 5) // Default of 4% (Bits 7:5)
#define RECEIVER_BW_MANT_16 (unsigned)0x00            // (Bits 4:3)
#define RECEIVER_BW_MANT_20 (unsigned)0x08            // (Bits 4:3)
#define RECEIVER_BW_MANT_24 (unsigned)0x10            // (Bits 4:3)

// DIO_INTERRUPTS
#define INTERRUPT_TIMEOUT 0
#define INTERRUPT_PACKET_SENT 0 
#define INTERRUPT_PAYLOAD_READY_TX_READY 1
// #define INTERRUPT_RX_READY 2

// DIO_MAPPING_1
#define DIO_0_MAPPING_M (unsigned)(3 << 6)  // (Bits 7:6) Mask
#define DIO_0_INTERRUPT(mapping) (unsigned)(mapping << 6)

// DIO_MAPPING_2
#define DIO_CLK_OUT_OFF (unsigned)(7 << 0) // (Bits 2:0)

#define DIO_4_INTERRUPT(MAPPING) (unsigned)(MAPPING << 6)

// IRQ_FLAGS_1
#define IRQ_1_MODE_READY (unsigned)(1 << 7) // Mode Ready Flag
#define IRQ_1_RX_READY (unsigned)(1 << 6) // RX Ready Flag
#define IRQ_1_TX_READY (unsigned)(1 << 5) // TX Ready Flag
#define IRQ_1_TIMEOUT (unsigned)(1 << 2)    // Timeout Flag

// IRQ_FLAGS_2
#define IRQ_2_PACKET_SENT (unsigned)(1 << 3)   // Packet Sent Flag in TX
#define IRQ_2_PAYLOAD_READY (unsigned)(1 << 2) // Payload Ready Flag in RX/Standby
#define IRQ_2_CRC_OK (unsigned)(1 << 1)        // Entering Intermediate Mode Flag

// TEST_DAGC
#define TEST_DAGC_IMPROVED_AFC_0 0x30

// CUSTOM ACK_STATUS
#define ACK_RESET (unsigned)0x00
#define ACK_PAYLOAD_PASSED (unsigned)0x02
#define ACK_PAYLOAD_RECEIVED (unsigned)0x01

#define CTL_IS_ACK 0x01

#define MetadataLength 5       // Contains Payload Length, Destination Node ID, Source Node ID, tracking and ACK bytes in that order
#define MetadataLength2Bytes 3 // = ((MetadataLength / 2) + 1 / 2)
#define RFM69HCW_INT_PRIORITY 1

#define PACKET_MAX_RETRIES 3
#define PACKET_SENT_MAX_TIME (unsigned)1e6 // 80MHz * 500ms
#define PACKET_ACK_MAX_TIME  (unsigned)1e6 // 80MHz * 500ms

extern bool HasNewData;
extern uint8_t RX_Data_Metadata[MetadataLength];
extern uint8_t RX_Data_Buffer[PAYLOAD_LENGTH_64 + 1];

void RFM69HCW_Init(uint32_t SYS_CLK, uint32_t SSI_CLK);

bool RFM69HCW_SendPacket(uint8_t* data, uint8_t length, bool isACK);

void RFM69HCW_PrintRSSI(void);

void RFM69HCW_ReadRegisterCLI(void);
