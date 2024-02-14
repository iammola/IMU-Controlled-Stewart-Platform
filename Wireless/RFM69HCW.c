#include "RFM69HCW.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "CLI/CLI.h"
#include "SPI/SPI.h"
#include "SysTick.h"
#include "tm4c123gh6pm.h"

#define DIO_0_INT_BIT (unsigned)(1 << 2) // PB2
#define DIO_0_PCTL_M  (unsigned)GPIO_PCTL_PB2_M
#define DIO_4_INT_BIT (unsigned)(1 << 3) // PB3
#define DIO_4_PCTL_M  (unsigned)GPIO_PCTL_PB3_M

#define RESET_BIT    (unsigned)(1 << 5) // PA5
#define RESET_PCTL_M (unsigned)(GPIO_PCTL_PA5_M)

#define INT_PINS   (unsigned)(DIO_0_INT_BIT)
#define INT_PCTL_M (unsigned)(DIO_0_PCTL_M)

#define READ(addr)        (uint16_t)(0x7FFF & (addr << 8))
#define WRITE(addr, data) (uint16_t)(0x8000 | (addr << 8) | data)

#define NETWORK_ID 23

void UART0_Handler(void);
void GPIOB_Handler(void);

static void RFM69HCW_Config(uint32_t bitRate, uint32_t deviation, uint8_t rxBW);
static void RFM69HCW_ClearCLIBuffer(void);
static void RFM69HCW_Interrupt_Init(void);
static void RFM69HCW_SetMode(MODE newMode, bool EnableListenMode);
static void RFM69HCW_SetNodesID(void);
static MODE RFM69HCW_GetMode(void);
static void RFM69HCW_Reset(void);

static void    RFM69HCW_WriteRegister(ADDRESS REGISTER, uint8_t data);
static uint8_t RFM69HCW_ReadRegister(ADDRESS REGISTER);

static uint8_t LAST_SENT_ACK = 0;
static uint8_t deviceVersion = 0;
static uint8_t ACK_STATUS = ACK_RESET;

static const char AES_CIPHER_KEY[16] = "($HJ#BUCA823nGU1";

static uint8_t NodeID = 0;
static uint8_t PeerID = 0;

bool    HasNewData = false;
uint8_t RX_Data_Metadata[MetadataLength] = {0};
uint8_t RX_Data_Buffer[PAYLOAD_LENGTH_64 + 1] = {0};

#define MAX_CLI_TEXT_BUFFER 500
static char text[MAX_CLI_TEXT_BUFFER] = {0};

static MODE CurrentMode;
static bool HasInitResetPin = false;

static uint8_t  CLI_Idx = 0;
static uint16_t CLI_Address = 0;
static char     CLI_Buffer[255] = {0};
static bool     PerformCLIAction = false;

void UART0_Handler(void) {
  char typed = CLI_Read();
  UART0_ICR_R |= UART_ICR_RXIC | UART_ICR_RTIC;

  if (typed == 0x0D) {
    PerformCLIAction = true;
    return;
  }

  CLI_Buffer[CLI_Idx] = typed;
  CLI_Idx++;
}

void GPIOB_Handler(void) {
  uint8_t intStatus = RFM69HCW_ReadRegister(IRQ_FLAGS_2);

  // Check if it is a Timeout Interrupt
  //   if (GPIO_PORTB_MIS_R & DIO_4_INT_BIT) {
  //     GPIO_PORTB_ICR_R = DIO_4_INT_BIT;

  //     if (RFM69HCW_ReadRegister(IRQ_FLAGS_1) & IRQ_1_TIMEOUT) {
  //       CLI_Write(" RSSI signal passing threshold causing Timeout\n\r");

  //       // Restart RX to detect preamble of signal
  // //      RFM69HCW_WriteRegister(PACKET_CONFIG_2, RFM69HCW_ReadRegister(PACKET_CONFIG_2) | PACKET_RX_RESTART);
  //     }
  //   }

  if (!(GPIO_PORTB_MIS_R & DIO_0_INT_BIT)) {
    CLI_Write("\n\r Not DIO0 interrupt\n\r");
    return;
  }

  snprintf(text, MAX_CLI_TEXT_BUFFER, "\n\r\n\r MIS = %#04x\n\r Operation Mode = %#04x\n\r INT_2 = %#04x\n\r", GPIO_PORTB_MIS_R, CurrentMode,
           intStatus);
  CLI_Write(text);

  // If in RX mode and the Payload Ready Interrupt was set
  if (intStatus & IRQ_2_PAYLOAD_READY) {
    CLI_Write("\n\r Event: Received Payload Ready\n\r");

    // CLI_Write("\n\r Reading Metadata\n\r");
    // Read Metadata (Payload Length and Sender ID rn)
    uint8_t dataIdx = 0;
    for (; dataIdx < MetadataLength; dataIdx++) {
      RX_Data_Metadata[dataIdx] = RFM69HCW_ReadRegister(FIFO);
      snprintf(text, MAX_CLI_TEXT_BUFFER, " %d. %#04x", dataIdx + 1, RX_Data_Metadata[dataIdx]);
      CLI_Write(text);
    }
    // CLI_Write("\n\r Metadata End\n\r");

    if (RX_Data_Metadata[3] & CTL_IS_ACK) {
      CLI_Write(" ACK Payload Received\n\r");
      // Confirm Packet was received
      ACK_STATUS |= ACK_PAYLOAD_RECEIVED;

      if (RX_Data_Metadata[3] == LAST_SENT_ACK) {
        ACK_STATUS = ACK_PAYLOAD_PASSED;
        CLI_Write(" ACK Passed\n\r");
      }
    } else {
      HasNewData = true;

      // Remove Metadata Length from Total Payload Length
      RX_Data_Metadata[0] -= MetadataLength;

      // Read Data Stream
      // CLI_Write("\n\r Data Start\n\r");
      for (dataIdx = 0; dataIdx < RX_Data_Metadata[0]; dataIdx++) {
        RX_Data_Buffer[dataIdx] = RFM69HCW_ReadRegister(FIFO);
        snprintf(text, MAX_CLI_TEXT_BUFFER, " %d. %#04x", dataIdx + 1, RX_Data_Buffer[dataIdx]);
        CLI_Write(text);

        if (dataIdx > 0 && (dataIdx % 4) == 0)
          CLI_Write("\n\r");
      }
      // CLI_Write("\n\r Data End\n\r");

      // Null at the end of data
      RX_Data_Buffer[RX_Data_Metadata[0]] = 0;

      snprintf(text, MAX_CLI_TEXT_BUFFER, " Sending ACK = %d\n\r", RX_Data_Metadata[2]);
      CLI_Write(text);

      // Send ACK Packet
      RFM69HCW_SendPacket(&RX_Data_Metadata[2], 1, true);
    }
  }

  // Clear Interrupt
  GPIO_PORTB_ICR_R = DIO_0_INT_BIT;
}

static uint32_t MeasuredRSSIMin = 0;
static uint32_t MeasuredRSSIMax = 0;
static uint32_t MeasuredRSSICur = 0;
static bool     ClearRSSIPrintLine = false;

void RFM69HCW_PrintRSSI(void) {
  // Trigger RSSI sampling
  RFM69HCW_WriteRegister(RSSI_CONFIG, RSSI_CONFIG_START_SAMPLE);

  // Wait for sampling to be finished
  while (RFM69HCW_ReadRegister(RSSI_CONFIG) != RSSI_CONFIG_RESULT_AVAILABLE)
    ;

  MeasuredRSSICur = RFM69HCW_ReadRegister(RSSI_VALUE) / 2;
  if (ClearRSSIPrintLine)
    CLI_Write("\033[K\033[100D");
  else {
    ClearRSSIPrintLine = true;
    MeasuredRSSIMax = MeasuredRSSICur;
    MeasuredRSSIMin = MeasuredRSSICur;
  }

  snprintf(text, MAX_CLI_TEXT_BUFFER, " Min = -%ddBm, Max = -%ddBm, RSSI = -%ddBm", MeasuredRSSIMin, MeasuredRSSIMax, MeasuredRSSICur);
  CLI_Write(text);

  if (MeasuredRSSICur > MeasuredRSSIMax) {
    MeasuredRSSIMax = MeasuredRSSICur;
  }

  if (MeasuredRSSICur < MeasuredRSSIMin) {
    MeasuredRSSIMin = MeasuredRSSICur;
  }
}

void RFM69HCW_ReadRegisterCLI(void) {
  if (PerformCLIAction) {
    PerformCLIAction = false;
    CLI_Address = (uint16_t)strtol(CLI_Buffer, NULL, 0);

    snprintf(text, MAX_CLI_TEXT_BUFFER, "\n\r Requested Address = %#04x, READ = %#04x\n\r", CLI_Address, RFM69HCW_ReadRegister((ADDRESS)CLI_Address));
    CLI_Write(text);
    RFM69HCW_ClearCLIBuffer();
  }

  if (NodeID == 1) {
    uint8_t payload[] = "HOPPER";
    RFM69HCW_SendPacket(payload, 6, false);
  }
}

static void RFM69HCW_SetNodesID(void) {
  CLI_Write(" Enter this Node's Network ID \n\r");
  while (!PerformCLIAction)
    ;

  PerformCLIAction = false;
  NodeID = (uint8_t)strtol(CLI_Buffer, NULL, 0);

  snprintf(text, MAX_CLI_TEXT_BUFFER, " Node ID set to %#04x\n\r", NodeID);
  CLI_Write(text);
  RFM69HCW_ClearCLIBuffer();

  CLI_Write("\n\r Enter the Peer Node's Network ID \n\r");
  while (!PerformCLIAction)
    ;

  PerformCLIAction = false;
  PeerID = (uint8_t)strtol(CLI_Buffer, NULL, 0);

  snprintf(text, MAX_CLI_TEXT_BUFFER, " Peer Node ID set to %#04x\n\r", PeerID);
  CLI_Write(text);
  RFM69HCW_ClearCLIBuffer();

  CLI_Write("\n\r Press Enter to Continue.");
  while (!PerformCLIAction)
    ;
  CLI_Write("\n\r");
  PerformCLIAction = false;
}

static void RFM69HCW_ClearCLIBuffer(void) {
  while (CLI_Idx > 0) {
    CLI_Buffer[--CLI_Idx] = '\0';
  }
}

static void RFM69HCW_Interrupt_Init(void) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // Enable Port B clock

  GPIO_PORTB_AFSEL_R &= ~INT_PINS;  // Disable Alternate Functions on interrupt pins
  GPIO_PORTB_PCTL_R &= ~INT_PCTL_M; // Disable Peripheral functions on the interrupt pins
  GPIO_PORTB_DIR_R &= ~INT_PINS;    // Configure interrupt pins as inputs
  GPIO_PORTB_DEN_R |= INT_PINS;     // Enable Digital Mode on interrupt pins
  GPIO_PORTB_AMSEL_R &= ~INT_PINS;  // Disable Analog Mode on interrupt pins
  GPIO_PORTB_IM_R &= ~INT_PINS;     // Disable interrupt mask on interrupt pins
  GPIO_PORTB_IS_R &= ~INT_PINS;     // Configure for Edge-Detect interrupts
  GPIO_PORTB_IBE_R &= ~INT_PINS;    // Only listen on one edge event on the pin
  GPIO_PORTB_IEV_R |= INT_PINS;     // Trigger interrupt on rising edge

  NVIC_EN0_R |= NVIC_EN0_INT1;                                                                             // Enable Port B's Interrupt Handler
  NVIC_PRI0_R = (NVIC_PRI0_R & (unsigned)~NVIC_PRI0_INT1_M) | (RFM69HCW_INT_PRIORITY << NVIC_PRI0_INT1_S); // Configure Port B's priority

  GPIO_PORTB_ICR_R |= INT_PINS; // Clear the pins interrupt
  GPIO_PORTB_IM_R |= INT_PINS;  // Allow Interrupts on the pins to be detected
}

static void RFM69HCW_Config(uint32_t bitRate, uint32_t deviation, uint8_t rxBW) {
  // Ensure deviation is more than half the bit-rate.
  if (deviation < (bitRate / 2))
    while (1)
      ;

  deviceVersion = RFM69HCW_ReadRegister(VERSION);
  snprintf(text, MAX_CLI_TEXT_BUFFER, " Device Version=%#04x\n\r", deviceVersion);
  CLI_Write(text);

  if (deviceVersion != 0x24)
    CLI_Write("\n\r Invalid Device Version \n\r");

  // Set Data Bit-Rate
  bitRate = (uint32_t)(F_XOSC / bitRate);
  RFM69HCW_WriteRegister(BITRATE_FIRST_BYTE, (bitRate & 0xFF00) >> 8);
  RFM69HCW_WriteRegister(BITRATE_LAST_BYTE, bitRate & 0xFF);

  // Set Frequency Deviation
  deviation = (uint32_t)(bitRate / (2 * F_STEP));
  RFM69HCW_WriteRegister(DEVIATION_FIRST_BYTE, (deviation >> 8) & 0x3F);
  RFM69HCW_WriteRegister(DEVIATION_LAST_BYTE, bitRate & 0xFF);

  // Set Carrier Frequency
  uint32_t carrierFrequency = (uint32_t)(MODULE_FREQUENCY / F_STEP);
  RFM69HCW_WriteRegister(CARRIER_FREQUENCY_FIRST_BYTE, (carrierFrequency & 0xFF0000) >> 16);
  RFM69HCW_WriteRegister(CARRIER_FREQUENCY_MID_BYTE, (carrierFrequency & 0xFF00) >> 8);
  RFM69HCW_WriteRegister(CARRIER_FREQUENCY_LAST_BYTE, carrierFrequency & 0xFF);

  // Use FSK modulation, packet data mode and no modulation shaping
  RFM69HCW_WriteRegister(DATA_MODULATION, DATA_MODULATION_NO_SHAPING | DATA_MODULATION_FSK | DATA_MODULATION_MODE);
  // Enable sync word verification, with 3 words and no tolerance for errors
  RFM69HCW_WriteRegister(SYNC_CONFIG, SYNC_WORD_VERIFICATION | SYNC_WORD_BYTE_COUNT_2 | SYNC_WORD_NO_TOLERANCE);
  RFM69HCW_WriteRegister(SYNC_VALUE_1, deviceVersion);
  RFM69HCW_WriteRegister(SYNC_VALUE_2, NETWORK_ID);
  // Configure the Packets to be of Variable Length, with Variable lengths and the addresses to be filtered for this node
  RFM69HCW_WriteRegister(PACKET_CONFIG_1, PACKET_VARIABLE_LENGTH | PACKET_DC_FREE_WHITENING /* | PACKET_CRC_ENABLE | PACKET_ADDRESS_FILTER_NODE */ |
                                              PACKET_CRC_AUTO_CLEAR_OFF);
  RFM69HCW_WriteRegister(PACKET_NODE_ADDR, NodeID);
  RFM69HCW_WriteRegister(FIFO_THRESHOLD, FIFO_TX_ON_NOT_EMPTY | FIFO_THRESHOLD_LEVEL_DEFAULT); // Trigger transmit start on non-empty FIFO buffer
  RFM69HCW_WriteRegister(PAYLOAD_LENGTH, PAYLOAD_LENGTH_64);                                   // Set (max) payload length to 64
  RFM69HCW_WriteRegister(RX_BANDWIDTH, RECEIVER_DC_OFFSET_CUTOFF_FREQ | rxBW);                 // Specify DC Offset to 4% of BW
  RFM69HCW_WriteRegister(RX_AFC_BANDWIDTH, RECEIVER_DC_OFFSET_CUTOFF_FREQ | rxBW);             // Specify DC Offset to 4% of BW
  RFM69HCW_WriteRegister(PA_RAMP_TIME, PA_FSK_RAMP_TIME_40u);                                  // Set Ramp-Time
  RFM69HCW_WriteRegister(PA_LEVEL, PA1_ON | PA_MAX_POWER);                                     // Enable PA1 and use the max power (13dBm)
  RFM69HCW_WriteRegister(CURRENT_PROTECTION, CURRENT_PROTECTION_ON); // Enable Over Current Protection (required for high power)
  RFM69HCW_WriteRegister(RSSI_THRESHOLD, 75);                        // Set RSSI Threshold
  RFM69HCW_WriteRegister(TIMEOUT_RX_START, 0);                       // Disable RX Timeout
  RFM69HCW_WriteRegister(TIMEOUT_RSSI_THRESHOLD, 100);               // Enable RSSI Timeout
  // Enable AES encryption, automatic RX phase restart and specified Inter Packet RX Delay
  RFM69HCW_WriteRegister(PACKET_CONFIG_2, ~PACKET_AES_ENCRYPTION & (PACKET_AUTO_RX_RESTART | PACKET_INTER_RX_DELAY_NONE));

  // Set Cipher Key
  uint16_t idx = 0;
  for (; idx < AES_KEY_LAST - AES_KEY_FIRST; idx++) {
    RFM69HCW_WriteRegister((ADDRESS)(AES_KEY_FIRST + idx), 0x00 /*AES_CIPHER_KEY[idx]*/);
  }

  // Set the resolution of the Idle and RX to be the same, for only the RSSI to be matched, and to resume in Listen Mode after
  RFM69HCW_WriteRegister(LISTEN_1, LISTEN_CRITERIA_THRESHOLD_RSSI | LISTEN_END_IDLE_RESUME | (2 << 6) | (2 << 4));
  RFM69HCW_WriteRegister(LISTEN_2, 0x25);                                                      // Give 25% of 1s/Resolution to Idle
  RFM69HCW_WriteRegister(LISTEN_3, 0xCF);                                                      // Give 75% of 1s/Resolution to RX
  RFM69HCW_WriteRegister(DIO_MAPPING_2, DIO_CLK_OUT_OFF | DIO_4_INTERRUPT(INTERRUPT_TIMEOUT)); // Disable Clock output and enable DIO4 Timeout
  RFM69HCW_WriteRegister(TEST_DAGC, TEST_DAGC_IMPROVED_AFC_0);                                 // Improved Fading Margin for AFC LowBeta = 0
  // Enable Payload Ready Interrupt for RX mode
  RFM69HCW_WriteRegister(DIO_MAPPING_1, DIO_0_INTERRUPT(INTERRUPT_PAYLOAD_READY_TX_READY));
  RFM69HCW_SetMode(OPERATION_MODE_STANDBY, true); // Enable Listen Mode
}

void RFM69HCW_Init(uint32_t SYS_CLK, uint32_t SSI_CLK) {
  // Initialize SysTick
  SysTick_Init();

  // Init UART COM
  CLI_Init(SYS_CLK, 115200, 3 /* UART_LCRH_WLEN_8 */, 5 /* UART_IFLS_RX4_8 */, 0x00 /* No Parity */, false);

  CLI_Write("\n\r----------------- RFM69HCW -----------------\n\r");

  do {
    // Configure Node IDs
    RFM69HCW_SetNodesID();
  } while (PeerID == 0 || NodeID == 0 || PeerID == NodeID);

  // Initialize the SPI pins
  SPI2_Init(SYS_CLK, SSI_CLK, SSI_CR0_FRF_MOTO, SSI_CR0_DSS_16);

  RFM69HCW_Reset(); // Reset Module

  CLI_Write("\n\r----------------- Config Start -----------------\n\r");
  // Configure Wireless settings
  // RFM69HCW_Config(384e2, 40e3, RECEIVER_BW_MANT_16 | 0);
  RFM69HCW_Config(250e3, 250e3, RECEIVER_BW_MANT_16 | 0);
  CLI_Write("\n\r----------------- Config End -----------------\n\r");

  snprintf(text, MAX_CLI_TEXT_BUFFER, " ID = %d, PEER = %d\n\r", NodeID, PeerID);
  CLI_Write(text);
  snprintf(text, MAX_CLI_TEXT_BUFFER, " DIO_MAPPING_1 Interrupt = %#04x\n\r DIO_MAPPING_2 Interrupt = %#04x\n\r",
           RFM69HCW_ReadRegister(DIO_MAPPING_1), RFM69HCW_ReadRegister(DIO_MAPPING_2));
  CLI_Write(text);
  CLI_Write("\n\r");
  // while(1);
  RFM69HCW_Interrupt_Init();
}

static void RFM69HCW_Reset(void) {
  CLI_Write("\n\r----------------- Reset Start -----------------\n\r");
  if (!HasInitResetPin) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; // Enable Port A clock
    do {
    } while (false);

    GPIO_PORTA_DIR_R |= RESET_BIT;      // Configure as output
    GPIO_PORTA_DEN_R |= RESET_BIT;      // Digital enable on pin
    GPIO_PORTA_AMSEL_R &= ~RESET_BIT;   // Analog disable on pin
    GPIO_PORTA_AFSEL_R &= ~RESET_BIT;   // Clear Digital alternate func
    GPIO_PORTA_PCTL_R &= ~RESET_PCTL_M; // Clear Pin Peripheral func

    HasInitResetPin = true;
  }

  GPIO_PORTA_DATA_R |= RESET_BIT;
  SysTick_Wait(8e3); // Pulled High for a 100us
  GPIO_PORTA_DATA_R &= ~RESET_BIT;
  SysTick_Wait(400e3); // Wait for at least 5ms
  CLI_Write("\n\r----------------- Reset End -----------------\n\r");
}

static uint8_t RFM69HCW_ReadRegister(ADDRESS REGISTER) {
  uint16_t response = 0;

  SPI2_StartTransmission();
  SPI2_Read(READ(REGISTER), &response, 1);
  SPI2_EndTransmission();

  return response & 0xFF;
}

static void RFM69HCW_WriteRegister(ADDRESS REGISTER, uint8_t data) {
  uint16_t byte = WRITE(REGISTER, data);
  bool     ShouldPrintDebug = RSSI_CONFIG != REGISTER && FIFO != REGISTER && (REGISTER != PACKET_CONFIG_2 || !(data & PACKET_RX_RESTART));

  if (ShouldPrintDebug) {
    snprintf(text, MAX_CLI_TEXT_BUFFER, " ADDR=%#04x,", REGISTER);
    CLI_Write(text);

    snprintf(text, MAX_CLI_TEXT_BUFFER, " WRITE=%#04x,", data);
    CLI_Write(text);
  }

  SPI2_StartTransmission();
  SPI2_Write(&byte, 1);
  SPI2_EndTransmission();

  if (ShouldPrintDebug) {
    snprintf(text, MAX_CLI_TEXT_BUFFER, " READ=%#04x\n\r", RFM69HCW_ReadRegister(REGISTER));
    CLI_Write(text);
  }
}

static MODE RFM69HCW_GetMode(void) {
  return RFM69HCW_ReadRegister(OPERATION_MODE) & OPERATION_MODE_M;
}

static void RFM69HCW_SetMode(MODE newMode, bool EnableListenMode) {
  // Get current operation mode settings
  uint8_t modeSettings = RFM69HCW_ReadRegister(OPERATION_MODE) & ~OPERATION_SEQUENCER_OFF;

  // Have to abort Listen Mode to change Operation mode
  // If the listen mode is not already enabled
  if (modeSettings & OPERATION_LISTEN_ON) {
    modeSettings = (modeSettings & ~OPERATION_MODE_M) | (EnableListenMode ? OPERATION_MODE_STANDBY : newMode);

    // First stage
    modeSettings = (modeSettings | OPERATION_LISTEN_ABORT) & ~OPERATION_LISTEN_ON;
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);

    // Second stage
    modeSettings &= ~OPERATION_LISTEN_ABORT;
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);
  } else {
    modeSettings = (modeSettings & ~OPERATION_MODE_M) | newMode;
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);
  }

  if (EnableListenMode) {
    modeSettings |= OPERATION_LISTEN_ON;
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);
  }

  // Wait for MODE_READY
  while ((RFM69HCW_ReadRegister(IRQ_FLAGS_1) & IRQ_1_MODE_READY) != IRQ_1_MODE_READY)
    ;

  CurrentMode = newMode;

  snprintf(text, MAX_CLI_TEXT_BUFFER, "\n\r Operation Mode = %#04x\n\r", RFM69HCW_ReadRegister(OPERATION_MODE));
  CLI_Write(text);
}

bool RFM69HCW_SendPacket(uint8_t *data, uint8_t length, bool isACK) {
  int32_t countdown;
  int8_t  maxRetries = PACKET_MAX_RETRIES;

  uint8_t  dataIdx = 0;
  uint8_t  payloadIdx = 0;
  uint16_t TX_Payload[PAYLOAD_LENGTH_64 / 2] = {0};

  CLI_Write("\n\r----------------- Send Packet Start -----------------\n\r");

  // Prevent RX while setting up TX
  RFM69HCW_SetMode(OPERATION_MODE_STANDBY, false);

  // Set Address and Payload Length as first bytes to transfer
  TX_Payload[payloadIdx++] = WRITE(FIFO, (length + MetadataLength));

  // Set the destination and source ID
  TX_Payload[payloadIdx++] = (uint16_t)((PeerID << 8) | NodeID);

  // Set the control and ACK to return in metadata
  TX_Payload[payloadIdx++] = (uint16_t)((isACK ? 0 : CTL_IS_ACK << 8) | ++LAST_SENT_ACK);

  // The total payload size would be the number of metadata bytes added so far to a 1-index form,
  // And the length of the actualy message in half + 0.5 to round up in case of odd lengths.
  uint8_t payloadSize = (payloadIdx + 1) + (length / 2) + (1 / 2);

  // Prevent sending more than max bytes
  if (payloadSize > (PAYLOAD_LENGTH_64 / 2)) {
    CLI_Write("Data is too big for FIFO");
    return false;
  }

  snprintf(text, MAX_CLI_TEXT_BUFFER, "  - Total Payload Size = %d bytes\n\r", TX_Payload[0] & 0xFF);
  CLI_Write(text);
  snprintf(text, MAX_CLI_TEXT_BUFFER, "  - Destination Node ID = %d\n\r", (TX_Payload[1] & 0xFF00) >> 8);
  CLI_Write(text);
  snprintf(text, MAX_CLI_TEXT_BUFFER, "  - Source Node ID = %d\n\r", (TX_Payload[1] & 0xFF));
  CLI_Write(text);
  snprintf(text, MAX_CLI_TEXT_BUFFER, "  - Control = %#04x\n\r", (TX_Payload[2] & 0xFF00) >> 8);
  CLI_Write(text);
  snprintf(text, MAX_CLI_TEXT_BUFFER, "  - Sent ACK = %d\n\r", (TX_Payload[2] & 0xFF));
  CLI_Write(text);
  snprintf(text, MAX_CLI_TEXT_BUFFER, "\n\r Data Payload = %d bytes\n\r\n\r", length);
  CLI_Write(text);

  for (; payloadIdx < payloadSize; payloadIdx++) {
    // Set the first byte in the 16 bit merge
    TX_Payload[payloadIdx] = (uint16_t)(data[dataIdx++] << 8);

    // Add the second byte if there's more to add
    if ((dataIdx + 1) <= length)
      TX_Payload[payloadIdx] |= data[dataIdx++];

    snprintf(text, MAX_CLI_TEXT_BUFFER, " DoubleWord #%d (%d = %#04x. %d = %#04x)\n\r", payloadIdx - MetadataLength2Bytes + 1, dataIdx - 1,
             (TX_Payload[payloadIdx] & 0xFF00) >> 8, dataIdx, TX_Payload[payloadIdx] & 0xFF);
    CLI_Write(text);
  }

  CLI_Write("\n\r Data End.\n\r");

  do {
    CLI_Write("\n\r Starting Transmission.\n\r");

    SPI2_StartTransmission();
    SPI2_Write(TX_Payload, payloadSize);
    SPI2_EndTransmission();

    CLI_Write(" Ending Transmission.\n\r");

    // Start Sending Packet from first byte in FIFO
    RFM69HCW_SetMode(OPERATION_MODE_TX, false);

    // Wait for PacketSent event
    countdown = PACKET_SENT_MAX_TIME;
    while (((RFM69HCW_ReadRegister(IRQ_FLAGS_2) & IRQ_2_PACKET_SENT) == 0x00) && SysTick_Countdown(&countdown))
      ;

    if (countdown < 0) {
      CLI_Write(" Did not receive Packet Sent in desired time.\n\r");
      return false;
    }

    CLI_Write(" Received Packet Sent.\n\r");

    // Enable Listen Mode
    RFM69HCW_SetMode(OPERATION_MODE_STANDBY, true);

    /* if (isACK) */ break;

    // Wait for ACK confirmation in Interrupt Handler
    ACK_STATUS = ACK_RESET;
    countdown = PACKET_ACK_MAX_TIME;
    while (((ACK_STATUS & ACK_PAYLOAD_RECEIVED) != ACK_PAYLOAD_RECEIVED) && SysTick_Countdown(&countdown))
      ;

    if (countdown < 0) {
      CLI_Write(" Timed out while waiting for ACK\n\r");
      return false;
    }
    // Restart Transmission if the ACK was received but didn't match
  } while (((ACK_STATUS & ACK_PAYLOAD_PASSED) != ACK_PAYLOAD_PASSED) && SysTick_Countdown(&maxRetries));

  if (maxRetries <= 0) {
    CLI_Write(" Max-Number of retries reached\n\r");
    return false;
  }

  CLI_Write("\n\r----------------- Send Packet End -----------------\n\r");
  return true;
}
