#define MODULE_FREQUENCY 915e6      // 915 MHz (Marked on board) (890MHz - 1020 MHz)
#define F_XOSC 32e6                 // 32MHz (Table 2.3.2)
#define F_STEP (F_XOSC / (1 << 19)) // (Section 3.2.3.3)

typedef enum ADDRESSES
{
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
  VERSION = 0x10,
  /* TRANSMITTER REGISTERS */
  PA_LEVEL = 0x11,
  PA_RAMP_TIME = 0x12,
  CURRENT_PROTECTION = 0X13,
  /* RECEIVER REGISTERS */
  RX_BANDWIDTH = 0x19,
  /* IRQ REGISTERS */
  DIO_MAPPING_1 = 0x25,
  DIO_MAPPING_2 = 0x26,
  IRQ_FLAGS_1 = 0x27,
  IRQ_FLAGS_2 = 0x28,
  RSSI_THRESHOLD = 0x29,
  /* PACKET ENGINE REGISTERS */
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
} ADDRESS;

// OPERATION_MODE
typedef enum MODES
{
  OPERATION_MODE_SLEEP = 0x00,
  OPERATION_MODE_STANDBY = 0x04,
  OPERATION_MODE_FS = 0x08,
  OPERATION_MODE_TX = 0x0C,
  OPERATION_MODE_RX = 0x10,
} MODE;
#define OPERATION_MODE_M 0x1C // Mask
#define OPERATION_LISTEN_ON 0x40
#define OPERATION_SEQUENCER_OFF 0x80

// DATA_MODULATION
#define DATA_MODULATION_NO_SHAPING 0x00 // (Bits 1:0)
#define DATA_MODULATION_FSK 0x00        // (Bits 4:3)
#define DATA_MODULATION_MODE 0x00       // (Bits 6:5)

// LISTEN_1
#define LISTEN_END_NO_ACTION 0 << 1              // (Bits 2:1)
#define LISTEN_CRITERIA_THRESHOLD_ADDRESS 1 << 3 // (Bit 3)
#define LISTEN_RESO_IRX_DEFAULT 1 << 4                   // (Bits 5:4) default
#define LISTEN_RESO_IDLE_DEFAULT 2 << 6                  // (Bits 7:6) default

// RSSI_THRESHOLD
#define RSSI_THRESHOLD_DEFAULT 0xE4 // Default = 114dBm

// SYNC_CONFIG
#define SYNC_WORD_VERIFICATION 0x80
#define SYNC_WORD_BYTE_COUNT_3 0x10 // (Bits 5:3)
#define SYNC_WORD_NO_TOLERANCE 0x00 // (Bits 2:0)

// PACKET_CONFIG_1
#define PACKET_VARIABLE_LENGTH 0x80
#define PACKET_CRC_ENABLE 0x10
#define PACKET_CRC_AUTO_CLEAR_OFF 0x08  // (Bit 3)
#define PACKET_ADDRESS_FILTER_NODE 0x02 // (Bits 2:1)

// PAYLOAD_LENGTH
#define PAYLOAD_LENGTH_64 0x40

// FIFO_THRESHOLD
#define FIFO_TX_ON_NOT_EMPTY 0x80

// PACKET_CONFIG_2
#define PACKET_AES_ENCRYPTION 0x01
#define PACKET_AUTO_RX_RESTART 0x02
#define PACKET_INTER_RX_DELAY_S 4 // Shift amount for bits 7:4

// PA_LEVEL
#define PA0_ON 0x80
#define PA1_ON 0x40
#define PA2_ON 0x20
#define PA_MAX_POWER 0x1F

// CURRENT_PROTECTION
#define CURRENT_PROTECTION_ON 0x1A

// PA_RAMP_TIME
#define PA_FSK_RAMP_TIME_40u 0x09 // Default
#define PA_FSK_RAMP_TIME_20u 0x0C // For 20us

// RX_BANDWIDTH
#define RECEIVER_DC_OFFSET_CUTOFF_FREQ 0x40 // Default of 4% (Bits 7:5)
#define RECEIVER_BW_MANT_16 0x00            // (Bits 4:3)
#define RECEIVER_BW_MANT_20 0x08            // (Bits 4:3)
#define RECEIVER_BW_MANT_24 0x10            // (Bits 4:3)

// DIO_MAPPING_1
#define DIO_0_MAPPING_00 0 << 6 // (Bits 7:6) Packet Sent in TX
#define DIO_0_MAPPING_01 1 << 6 // (Bits 7:6) Payload Ready in RX
#define DIO_2_MAPPING_11 3 << 2 // (Bits 3:2) Auto Mode in any mode

// DIO_MAPPING_2
#define DIO_CLK_OUT_OFF 0x07

// IRQ_FLAGS_1
#define IRQ_1_MODE_READY 1 << 7 // Mode Ready Flag
#define IRQ_1_AUTO_MODE 1 << 1  // Entering Intermediate Mode Flag

// IRQ_FLAGS_2
#define IRQ_2_PACKET_SENT 1 << 3   // Packet Sent Flag in TX
#define IRQ_2_PAYLOAD_READY 1 << 2 // Payload Ready Flag in RX/Standby
#define IRQ_2_CRC_OK 1 << 1        // Entering Intermediate Mode Flag

// CUSTOM ACK_STATUS
#define ACK_RESET 0x00
#define ACK_PAYLOAD_PASSED 0x02
#define ACK_PAYLOAD_RECEIVED 0x01

#define MetadataLength 3 // Contains Payload Length, Sender Node ID, and ACK to return
#define RFM69HCW_INT_PRIORITY 1

extern bool HasNewData;
extern uint8_t RX_Data_Metadata[MetadataLength];
extern uint8_t RX_Data_Buffer[PAYLOAD_LENGTH_64 + 1];

void RFM69HCW_Init(uint32_t SYS_CLK, uint32_t SSI_CLK);

void RFM69HCW_SendPacket(uint8_t *data, uint8_t length);
