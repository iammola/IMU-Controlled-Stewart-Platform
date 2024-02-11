#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "SysTick.h"
#include "tm4c123gh6pm.h"

#include "SPI/SPI.h"
#include "RFM69HCW.h"
#include "CLI/CLI.h"

#define DIO_0_INT_BIT (unsigned)(1 << 2) // PB2
#define DIO_0_PCTL_M (unsigned)GPIO_PCTL_PB2_M
#define DIO_4_INT_BIT (unsigned)(1 << 3) // PB3
#define DIO_4_PCTL_M (unsigned)GPIO_PCTL_PB3_M

#define INT_PINS (unsigned)(DIO_0_INT_BIT | DIO_4_INT_BIT)
#define INT_PCTL_M (unsigned)(DIO_0_PCTL_M | DIO_4_PCTL_M)

#define READ(addr) (uint16_t)(0x7FFF & (addr << 8))
#define WRITE(addr, data) (uint16_t)(0x8000 | (addr << 8) | data)

#define NETWORK_ID 23

void UART0_Handler(void);
void GPIOB_Handler(void);

static void RFM69HCW_Config(uint32_t bitRate, uint32_t deviation, uint8_t rxBW, uint8_t interPacketRxDelay, uint8_t rampTime);
static void RFM69HCW_ClearCLIBuffer(void);
static void RFM69HCW_Interrupt_Init(void);
static void RFM69HCW_SetMode(MODE newMode, bool EnableListenMode);
static void RFM69HCW_SetNodesID(void);
static MODE RFM69HCW_GetMode(void);

static void RFM69HCW_WriteRegister(ADDRESS REGISTER, uint8_t data);
static uint8_t RFM69HCW_ReadRegister(ADDRESS REGISTER);

static uint8_t LAST_SENT_ACK = 0;
static uint8_t deviceVersion = 0;
static uint8_t ACK_STATUS = ACK_RESET;

static const char AES_CIPHER_KEY[16] = "($HJ#BUCA823nGU1";

static uint8_t NodeID = 0;
static uint8_t PeerID = 0;

bool HasNewData = false;
uint8_t RX_Data_Metadata[MetadataLength] = {0};
uint8_t RX_Data_Buffer[PAYLOAD_LENGTH_64 + 1] = {0};

#define MAX_CLI_TEXT_BUFFER 500
static char text[MAX_CLI_TEXT_BUFFER] = {0};

static MODE CurrentMode;

static uint8_t CLI_Idx = 0;
static uint16_t CLI_Address = 0;
static char CLI_Buffer[255] = {0};
static bool PerformCLIAction = false;

void UART0_Handler(void)
{
  char typed = CLI_Read();
  UART0_ICR_R |= UART_ICR_RXIC | UART_ICR_RTIC;

  if (typed == 0x0D)
  {
    PerformCLIAction = true;
    return;
  }

  CLI_Buffer[CLI_Idx] = typed;
  CLI_Idx++;
}

void GPIOB_Handler(void)
{
  uint8_t dataIdx = 0;
  uint8_t metadataPlaceholder;
  uint8_t interruptStatus = 0;

  snprintf(text, MAX_CLI_TEXT_BUFFER, "\n\r MIS = %#04x \n\r", GPIO_PORTB_MIS_R);
  CLI_Write(text);
  // Check if it is a Timeout Interrupt
  if (GPIO_PORTB_MIS_R & DIO_4_INT_BIT)
  {
    GPIO_PORTB_ICR_R = DIO_4_INT_BIT;

    CLI_Write("\n\r RSSI signal passing threshold causing Timeout\n\r");

    snprintf(text, MAX_CLI_TEXT_BUFFER, "\n\r Interrupt Flags 1 = %#04x \n\r", RFM69HCW_ReadRegister(IRQ_FLAGS_1));
    CLI_Write(text);

    // Restart RX to detect preamble of signal
    RFM69HCW_WriteRegister(PACKET_CONFIG_2, RFM69HCW_ReadRegister(PACKET_CONFIG_2) | PACKET_RX_RESTART);
    RFM69HCW_PrintRSSI();
  }

  interruptStatus = RFM69HCW_ReadRegister(IRQ_FLAGS_2);

  snprintf(text, MAX_CLI_TEXT_BUFFER, "\n\r Mode = %04x. Interrupt Flags 2 = %#04x \n\r", RFM69HCW_GetMode(), interruptStatus);
  CLI_Write(text);

  if (!(GPIO_PORTB_MIS_R & DIO_0_INT_BIT))
  {
    CLI_Write("Not DIO0 interrupt");
    return;
  }

  // Must be DIO0 interrupt
  if (interruptStatus & IRQ_2_PACKET_SENT)
  {
    CLI_Write("\n\r Event: Packet Sent\n\r");
    // Enter RX for ACK
    RFM69HCW_SetMode(OPERATION_MODE_RX, false);
  }
  // While in RX mode and the CRC OK flag is set, then this should be the ACK payload
  else if (interruptStatus & IRQ_2_CRC_OK)
  {
    CLI_Write("\n\r Event: Received CRC OK\n\r");

    CLI_Write("\n\r Reading Metadata\n\r");

    // Read Metadata
    for (dataIdx = 0; dataIdx < MetadataLength; dataIdx++)
    {
      metadataPlaceholder = RFM69HCW_ReadRegister(FIFO);
      snprintf(text, MAX_CLI_TEXT_BUFFER, " %d. %#04x", dataIdx + 1, metadataPlaceholder);
      CLI_Write(text);
    }

    CLI_Write("\n\r Metadata End\n\r");

    CLI_Write(" \n\rChecking ACK\n\r");
    // Check if the Payload matches the Last Sent ACK
    if (RFM69HCW_ReadRegister(FIFO) == LAST_SENT_ACK)
    {
      ACK_STATUS = ACK_PAYLOAD_PASSED;
      CLI_Write(" ACK Passed\n\r");
    }

    CLI_Write(" ACK Payload Received\n\r");

    // Confirm Packet was received
    ACK_STATUS |= ACK_PAYLOAD_RECEIVED;
  }
  // If in RX mode and the Payload Ready Interrupt was set
  else if (interruptStatus & IRQ_2_PAYLOAD_READY)
  {
    HasNewData = true;

    CLI_Write("\n\r Reading Metadata\n\r");
    // Read Metadata (Payload Length and Sender ID rn)
    for (dataIdx = 0; dataIdx < MetadataLength; dataIdx++)
    {
      RX_Data_Metadata[dataIdx] = RFM69HCW_ReadRegister(FIFO);
      snprintf(text, MAX_CLI_TEXT_BUFFER, " %d. %#04x", dataIdx + 1, RX_Data_Metadata[dataIdx]);
      CLI_Write(text);
    }
    CLI_Write("\n\r Metadata End\n\r");

    // Remove Metadata Length from Total Payload Length
    RX_Data_Metadata[0] -= MetadataLength;

    // Read Data Stream
    CLI_Write("\n\r Data Start\n\r");
    for (dataIdx = 0; dataIdx < RX_Data_Metadata[0]; dataIdx++)
    {
      RX_Data_Buffer[dataIdx] = RFM69HCW_ReadRegister(FIFO);
      snprintf(text, MAX_CLI_TEXT_BUFFER, " %d. %#04x", dataIdx + 1, RX_Data_Buffer[dataIdx]);
      CLI_Write(text);

      if (dataIdx > 0 && (dataIdx % 4) == 0)
        CLI_Write("\n\r");
    }
    CLI_Write("\n\r Data End\n\r");

    // Null at the end of data
    RX_Data_Buffer[RX_Data_Metadata[0]] = 0;

    snprintf(text, MAX_CLI_TEXT_BUFFER, " Sending ACK = %d\n\r", RX_Data_Metadata[2]);
    CLI_Write(text);

    // Send Packet without ACK
    RFM69HCW_SendPacket(&RX_Data_Metadata[2], 1, false);

    CLI_Write(" Waiting for Packet Sent event \n\r");
    // Wait for PacketSent event
    while ((RFM69HCW_ReadRegister(IRQ_FLAGS_2) & IRQ_2_PACKET_SENT) == 0x00)
      ;
    CLI_Write(" Received Packet Sent for ACK \n\r");
  }
  else
  {
    CLI_Write(" Event not matched");
  }

  // Clear Interrupt
  GPIO_PORTB_ICR_R = DIO_0_INT_BIT;
}

static uint32_t MeasuredRSSIMin = 0;
static uint32_t MeasuredRSSIMax = 0;
static uint32_t MeasuredRSSICur = 0;
static bool ClearRSSIPrintLine = false;

void RFM69HCW_PrintRSSI(void)
{
  if (RFM69HCW_GetMode() == OPERATION_MODE_RX)
  {
    // Trigger RSSI sampling
    RFM69HCW_WriteRegister(RSSI_CONFIG, RSSI_CONFIG_START_SAMPLE);

    // Wait for sampling to be finished
    while (RFM69HCW_ReadRegister(RSSI_CONFIG) != RSSI_CONFIG_RESULT_AVAILABLE)
      ;

    MeasuredRSSICur = RFM69HCW_ReadRegister(RSSI_VALUE) / 2;
    if (ClearRSSIPrintLine)
      CLI_Write("\033[K\033[100D");
    else
    {
      ClearRSSIPrintLine = true;
      MeasuredRSSIMax = MeasuredRSSICur;
      MeasuredRSSIMin = MeasuredRSSICur;
    }

    snprintf(text, MAX_CLI_TEXT_BUFFER, " Min = -%ddBm, Max = -%ddBm, RSSI = -%ddBm", MeasuredRSSIMin, MeasuredRSSIMax, MeasuredRSSICur);
    CLI_Write(text);

    if (MeasuredRSSICur > MeasuredRSSIMax)
    {
      MeasuredRSSIMax = MeasuredRSSICur;
    }

    if (MeasuredRSSICur < MeasuredRSSIMin)
    {
      MeasuredRSSIMin = MeasuredRSSICur;
    }
  }
}

void RFM69HCW_ReadRegisterCLI(void)
{
  if (PerformCLIAction)
  {
    PerformCLIAction = false;
    CLI_Address = (uint16_t)strtol(CLI_Buffer, NULL, 0);

    snprintf(text, MAX_CLI_TEXT_BUFFER, "\n\r Requested Address = %#04x, READ = %#04x\n\r", CLI_Address, RFM69HCW_ReadRegister((ADDRESS)CLI_Address));
    CLI_Write(text);
    RFM69HCW_ClearCLIBuffer();
  }
}

static void RFM69HCW_SetNodesID(void)
{
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

static void RFM69HCW_ClearCLIBuffer(void)
{
  while (CLI_Idx > 0)
  {
    CLI_Buffer[--CLI_Idx] = '\0';
  }
}

static void RFM69HCW_Interrupt_Init(void)
{
  // Enable Port B clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;

  // Disable Alternate Functions on interrupt pins
  GPIO_PORTB_AFSEL_R &= ~INT_PINS;

  // Disable Peripheral functions on the interrupt pins
  GPIO_PORTB_PCTL_R &= ~INT_PCTL_M;

  // Configure interrupt pins as inputs
  GPIO_PORTB_DIR_R &= ~INT_PINS;

  // Enable Digital Mode on interrupt pins
  GPIO_PORTB_DEN_R |= INT_PINS;

  // Disable Analog Mode on interrupt pins
  GPIO_PORTB_AMSEL_R &= ~INT_PINS;

  // Disable interrupt mask on interrupt pins
  GPIO_PORTB_IM_R &= ~INT_PINS;

  // Configure for Edge-Detect interrupts
  GPIO_PORTB_IS_R &= ~INT_PINS;

  // Only listen on one edge event on the pin
  GPIO_PORTB_IBE_R &= ~INT_PINS;

  // Trigger interrupt on rising edge
  GPIO_PORTB_IEV_R |= INT_PINS;

  // Enable Port B's Interrupt Handler
  NVIC_EN0_R |= NVIC_EN0_INT1;

  // Configure Port B's priority
  NVIC_PRI0_R = (NVIC_PRI0_R & (unsigned)~NVIC_PRI0_INT1_M) | (RFM69HCW_INT_PRIORITY << NVIC_PRI0_INT1_S);

  // Clear the pins interrupt
  GPIO_PORTB_ICR_R |= INT_PINS;

  // Allow Interrupts on the pins to be detected
  GPIO_PORTB_IM_R |= INT_PINS;
}

static void RFM69HCW_Config(uint32_t bitRate, uint32_t deviation, uint8_t rxBW, uint8_t interPacketRxDelay, uint8_t rampTime)
{
  uint16_t idx = 0;
  uint32_t carrierFrequency = 0;

  // Ensure deviation is more than half the bit-rate.
  if (deviation < (bitRate / 2))
    while (1)
      ;

  deviceVersion = RFM69HCW_ReadRegister(VERSION);
  snprintf(text, MAX_CLI_TEXT_BUFFER, " Device Version=%#04x\n\r", deviceVersion);
  CLI_Write(text);

  // Use FSK modulation, packet data mode and no modulation shaping
  RFM69HCW_WriteRegister(DATA_MODULATION, DATA_MODULATION_NO_SHAPING | DATA_MODULATION_FSK | DATA_MODULATION_MODE);

  // Set Data Bit-Rate
  bitRate = (uint32_t)(F_XOSC / bitRate);
  RFM69HCW_WriteRegister(BITRATE_FIRST_BYTE, (bitRate & 0xFF00) >> 8);
  RFM69HCW_WriteRegister(BITRATE_LAST_BYTE, bitRate & 0xFF);

  // Set Frequency Deviation
  deviation = (uint32_t)(bitRate / (2 * F_STEP));
  RFM69HCW_WriteRegister(DEVIATION_FIRST_BYTE, (deviation >> 8) & 0x3F);
  RFM69HCW_WriteRegister(DEVIATION_LAST_BYTE, bitRate & 0xFF);

  // Set Carrier Frequency
  carrierFrequency = (uint32_t)(MODULE_FREQUENCY / F_STEP);
  RFM69HCW_WriteRegister(CARRIER_FREQUENCY_FIRST_BYTE, (carrierFrequency & 0xFF0000) >> 16);
  RFM69HCW_WriteRegister(CARRIER_FREQUENCY_MID_BYTE, (carrierFrequency & 0xFF00) >> 8);
  RFM69HCW_WriteRegister(CARRIER_FREQUENCY_LAST_BYTE, carrierFrequency & 0xFF);

  RFM69HCW_WriteRegister(PREAMBLE_LAST_BYTE, 16);

  // Enable sync word verification, with 3 words and no tolerance for errors
  RFM69HCW_WriteRegister(SYNC_CONFIG, SYNC_WORD_VERIFICATION | SYNC_WORD_BYTE_COUNT_2 | SYNC_WORD_NO_TOLERANCE);

  // Verify the Device Version, Network ID
  RFM69HCW_WriteRegister(SYNC_VALUE_1, deviceVersion);
  RFM69HCW_WriteRegister(SYNC_VALUE_2, NETWORK_ID);

  // Configure the Packets to be of Variable Length, with Variable lengths and the addresses to be filtered for this node
  RFM69HCW_WriteRegister(PACKET_CONFIG_1, PACKET_VARIABLE_LENGTH /* | PACKET_CRC_ENABLE | PACKET_ADDRESS_FILTER_NODE */ | PACKET_CRC_AUTO_CLEAR_OFF);
  RFM69HCW_WriteRegister(PACKET_NODE_ADDR, NodeID);

  // Trigger transmit start on non-empty FIFO buffer
  RFM69HCW_WriteRegister(FIFO_THRESHOLD, FIFO_TX_ON_NOT_EMPTY);

  // Set (max) payload length to 64
  RFM69HCW_WriteRegister(PAYLOAD_LENGTH, PAYLOAD_LENGTH_64);

  // Specify DC Offset to 4% of BW
  RFM69HCW_WriteRegister(RX_BANDWIDTH, RECEIVER_DC_OFFSET_CUTOFF_FREQ | rxBW);

  // Set Ramp-Time
  RFM69HCW_WriteRegister(PA_RAMP_TIME, rampTime & PA_FSK_RAMP_TIME_M);

  // Enable PA1 and use half of the max power (13dBm)
  RFM69HCW_WriteRegister(PA_LEVEL, PA1_ON | PA_MAX_POWER);

  // Enable Over Current Protection (required for high power)
  RFM69HCW_WriteRegister(CURRENT_PROTECTION, CURRENT_PROTECTION_ON);

  // Set RSSI Threshold
  RFM69HCW_WriteRegister(RSSI_THRESHOLD, 75);

  // Disable RX Timeout
  RFM69HCW_WriteRegister(TIMEOUT_RX_START, 0);
  
  // Enable RSSI Timeout
  RFM69HCW_WriteRegister(TIMEOUT_RSSI_THRESHOLD, 75);

  // Enable AES encryption, automatic RX phase restart and specified Inter Packet RX Delay
  RFM69HCW_WriteRegister(PACKET_CONFIG_2, ~PACKET_AES_ENCRYPTION & (PACKET_AUTO_RX_RESTART | PACKET_INTER_RX_DELAY_NONE));

  // Set Cipher Key
  for (idx = 0; idx < AES_KEY_LAST - AES_KEY_FIRST; idx++)
    RFM69HCW_WriteRegister((ADDRESS)(AES_KEY_FIRST + idx), 0x00 /*AES_CIPHER_KEY[idx]*/);

  // Set the resolution of the Idle and RX to be the same, for only the RSSI to be matched, and to resume in Listen Mode after
  RFM69HCW_WriteRegister(LISTEN_1, LISTEN_CRITERIA_THRESHOLD_RSSI | LISTEN_END_IDLE_RESUME | (2 << 6) | (2 << 4));
  // Give 25% of 1s/Resolution to Idle
  RFM69HCW_WriteRegister(LISTEN_2, 0x25);
  // Give 75% of 1s/Resolution to RX
  RFM69HCW_WriteRegister(LISTEN_3, 0xCF);

  // Disable Clock output and enable DIO4 Timeout
  RFM69HCW_WriteRegister(DIO_MAPPING_2, DIO_CLK_OUT_OFF | DIO_4_MAPPING_00);

  // Improved Fading Margin for AFC LowBeta = 0
  RFM69HCW_WriteRegister(TEST_DAGC, TEST_DAGC_IMPROVED_AFC_0);

  // Enable Listen Mode
  RFM69HCW_SetMode(OPERATION_MODE_STANDBY, true);
}

void RFM69HCW_Init(uint32_t SYS_CLK, uint32_t SSI_CLK)
{
  // Initialize SysTick
  SysTick_Init();

  // Init UART COM
  CLI_Init(SYS_CLK, 115200, 3 /* UART_LCRH_WLEN_8 */, 5 /* UART_IFLS_RX4_8 */, 0x00 /* No Parity */, false);

  CLI_Write("\n\r----------------- RFM69HCW -----------------\n\r");

  do
  {
    // Configure Node IDs
    RFM69HCW_SetNodesID();
  } while (PeerID == 0 || NodeID == 0 || PeerID == NodeID);

  // Initialize the SPI pins
  SPI2_Init(SYS_CLK, SSI_CLK, SSI_CR0_FRF_MOTO, SSI_CR0_DSS_16);

  CurrentMode = RFM69HCW_GetMode();

  CLI_Write("\n\r----------------- Config Start -----------------\n\r");
  // Configure Wireless settings
  RFM69HCW_Config(
      125e3 /* 125kHz Bit-rate */,
      75e3 /* Deviation */,
      RECEIVER_BW_MANT_24 | 1 /* Set RxBwExp to 1, and use 24 for RxBwMant */,
      4 /* 4 bit delay, for 125us Ramp */,
      PA_FSK_RAMP_TIME_125u /*  */
  );

  CLI_Write("\n\r----------------- Config End -----------------\n\r");
  snprintf(text, MAX_CLI_TEXT_BUFFER, " ID = %d, PEER = %d\n\r", NodeID, PeerID);
  CLI_Write(text);
  snprintf(text, MAX_CLI_TEXT_BUFFER, " DIO_MAPPING_1 Interrupt = %#04x\n\r", RFM69HCW_ReadRegister(DIO_MAPPING_1));
  CLI_Write(text);
  CLI_Write("\n\r");

  RFM69HCW_Interrupt_Init();
}

static uint8_t RFM69HCW_ReadRegister(ADDRESS REGISTER)
{
  uint16_t response = 0;

  SPI2_StartTransmission();
  SPI2_Read(READ(REGISTER), &response, 1);
  SPI2_EndTransmission();

  return response & 0xFF;
}

static void RFM69HCW_WriteRegister(ADDRESS REGISTER, uint8_t data)
{
  uint16_t byte = WRITE(REGISTER, data);
  bool ShouldPrintDebug = RSSI_CONFIG != REGISTER && FIFO != REGISTER && (REGISTER != PACKET_CONFIG_2 || !(data & PACKET_RX_RESTART));

  if (ShouldPrintDebug)
  {
    snprintf(text, MAX_CLI_TEXT_BUFFER, " ADDR=%#04x,", REGISTER);
    CLI_Write(text);

    snprintf(text, MAX_CLI_TEXT_BUFFER, " WRITE=%#04x,", data);
    CLI_Write(text);
  }

  SPI2_StartTransmission();
  SPI2_Write(&byte, 1);
  SPI2_EndTransmission();

  if (ShouldPrintDebug)
  {
    snprintf(text, MAX_CLI_TEXT_BUFFER, " READ=%#04x\n\r", RFM69HCW_ReadRegister(REGISTER));
    CLI_Write(text);
  }
}

static MODE RFM69HCW_GetMode(void)
{
  return RFM69HCW_ReadRegister(OPERATION_MODE) & OPERATION_MODE_M;
}

static void RFM69HCW_SetMode(MODE newMode, bool EnableListenMode)
{
  // Get current operation mode settings
  uint8_t modeSettings = RFM69HCW_ReadRegister(OPERATION_MODE) & ~OPERATION_SEQUENCER_OFF;

  // Have to abort Listen Mode to change Operation mode
  // If the listen mode is not already enabled
  if (modeSettings & OPERATION_LISTEN_ON)
  {
    if (EnableListenMode)
    {
      modeSettings = (modeSettings & ~OPERATION_MODE_M) | OPERATION_MODE_STANDBY;
    }

    // First stage
    modeSettings = (modeSettings | OPERATION_LISTEN_ABORT) & ~OPERATION_LISTEN_ON;
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);

    // Second stage
    modeSettings &= ~OPERATION_LISTEN_ABORT;
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);
  }

  if (EnableListenMode)
  {
    modeSettings |= OPERATION_LISTEN_ON;
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);
    // Enable Payload Ready Interrupt for RX mode
    RFM69HCW_WriteRegister(DIO_MAPPING_1, DIO_0_MAPPING_01);
  }
  else if (newMode != OPERATION_MODE_STANDBY)
  {
    modeSettings = (modeSettings & ~OPERATION_MODE_M) | newMode;
    // Set new Mode
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);
  }

  // Wait for MODE_READY
  while ((RFM69HCW_ReadRegister(IRQ_FLAGS_1) & IRQ_1_MODE_READY) != IRQ_1_MODE_READY)
    ;

  CurrentMode = newMode;

  snprintf(text, MAX_CLI_TEXT_BUFFER, " Operation Mode = %#04x\n\r\n\r", RFM69HCW_ReadRegister(OPERATION_MODE));
  CLI_Write(text);
}

void RFM69HCW_SendPacket(uint8_t *data, uint8_t length, bool waitForACK)
{
  uint8_t dataIdx = 0;
  uint8_t payloadIdx = 0;
  uint8_t payloadSize = MetadataLength2Bytes + (length / 2) + 0.5;
  uint16_t TX_Payload[MetadataLength2Bytes + (PAYLOAD_LENGTH_64 / 2)] = {0};

  CLI_Write("\n\r----------------- Send Packet Start -----------------\n\r");

  RFM69HCW_SetMode(OPERATION_MODE_STANDBY, false);

  // Prevent sending more than max bytes
  if (length > PAYLOAD_LENGTH_64)
  {
    CLI_Write(" Data is more than max size.\n\r");
    return;
  }

  // Set Address and Payload Length as first bytes to transfer
  TX_Payload[payloadIdx++] = WRITE(FIFO, (length + MetadataLength - 1));

  // Set the destination ID and ACK to return in metadata
  TX_Payload[payloadIdx++] = (PeerID << 8) | ++LAST_SENT_ACK;

  snprintf(text, MAX_CLI_TEXT_BUFFER, "  - Total Payload Size = %d bytes\n\r", TX_Payload[0] & 0xFF);
  CLI_Write(text);
  snprintf(text, MAX_CLI_TEXT_BUFFER, "  - Destination Node ID = %d\n\r", (TX_Payload[1] & 0xFF00) >> 8);
  CLI_Write(text);
  snprintf(text, MAX_CLI_TEXT_BUFFER, "  - Sent ACK = %d\n\r", (TX_Payload[1] & 0xFF));
  CLI_Write(text);
  snprintf(text, MAX_CLI_TEXT_BUFFER, "\n\r Data Payload = %d bytes\n\r\n\r", length);
  CLI_Write(text);

  for (; payloadIdx < payloadSize; payloadIdx++)
  {
    // Set the first byte in the 16 bit merge
    TX_Payload[payloadIdx] = (uint16_t)(data[dataIdx++] << 8);

    // Add the second byte if there's more to add
    if ((dataIdx + 1) <= length)
      TX_Payload[payloadIdx] |= data[dataIdx++];

    snprintf(text, MAX_CLI_TEXT_BUFFER, " DoubleWord #%d (%d = %#04x. %d = %#04x)\n\r", payloadIdx - MetadataLength2Bytes + 1, dataIdx - 1, (TX_Payload[payloadIdx] & 0xFF00) >> 8, dataIdx, TX_Payload[payloadIdx] & 0xFF);
    CLI_Write(text);
  }

  CLI_Write("\n\r Data End.\n\r");

  do
  {
    if (waitForACK)
    {
      // Enable Packet Sent for TX and CRC OK for RX Interrupt on DIO0
      RFM69HCW_WriteRegister(DIO_MAPPING_1, DIO_0_MAPPING_00);
    }

    // Start Sending Packet from first byte in FIFO
    RFM69HCW_SetMode(OPERATION_MODE_TX, false);

    CLI_Write("\n\r Starting Transmission.\n\r");

    SPI2_StartTransmission();
    SPI2_Write(TX_Payload, payloadIdx);
    SPI2_EndTransmission();

    CLI_Write(" Ending Transmission.\n\r");

    if (!waitForACK)
      break;

    // Wait for ACK confirmation in Interrupt Handler
    ACK_STATUS = ACK_RESET;
    while ((ACK_STATUS & ACK_PAYLOAD_RECEIVED) != ACK_PAYLOAD_RECEIVED)
      ;
    // Restart Transmission if the ACK was received but didn't match
  } while ((ACK_STATUS & ACK_PAYLOAD_PASSED) != ACK_PAYLOAD_PASSED);

  // Enable Listen Mode
  RFM69HCW_SetMode(OPERATION_MODE_STANDBY, true);

  CLI_Write("\n\r----------------- Send Packet End -----------------\n\r");
  SysTick_Wait10ms(75);
}
