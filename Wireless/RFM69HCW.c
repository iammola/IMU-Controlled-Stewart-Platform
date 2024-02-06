#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "SysTick.h"
#include "tm4c123gh6pm.h"

#include "SPI/SPI.h"
#include "RFM69HCW.h"

#define DIO_0_INT_BIT (unsigned)(1 << 0) // PB0

#define INT_PCTL_M (unsigned)GPIO_PCTL_PB0_M

#define READ(addr) (uint16_t)(0x7FFF & (addr << 8))
#define WRITE(addr, data) (uint16_t)(0x8000 | (addr << 8) | data)

#define NETWORK_ID 23

#ifdef __DEVICE_1__
#define MY_NODE_ID 54
#define PEER_NODE_ID 92
#else
#define MY_NODE_ID 92
#define PEER_NODE_ID 54
#endif

static void RFM69HCW_Config(uint32_t bitRate, uint32_t deviation, uint8_t rxBW, uint8_t interPacketRxDelay);
static void RFM69HCW_Interrupt_Init(void);
static void RFM69HCW_SetMode(MODE newMode);

static void RFM69HCW_WriteRegister(ADDRESS REGISTER, uint8_t data);
static uint8_t RFM69HCW_ReadRegister(ADDRESS REGISTER);

static uint8_t LAST_SENT_ACK = 0;
static uint8_t deviceVersion = 0;
static uint8_t ACK_STATUS = ACK_RESET;

static const char AES_CIPHER_KEY[16] = "($HJ#BUCA823nGU1";

bool HasNewData = false;
uint8_t RX_Data_Metadata[MetadataLength] = {0};
uint8_t RX_Data_Buffer[PAYLOAD_LENGTH_64 + 1] = {0};

static MODE CurrentMode;

void GPIOB_Handler(void)
{
  /*
    Tracks if the Interrupt was caused by a known event, and only clears it if so.
    Basically polling until the event that caused the interrupt is known
  */
  bool Handled = true;
  uint8_t dataIdx = 0;
  uint8_t interrupt2Status = RFM69HCW_ReadRegister(IRQ_FLAGS_2);

  if (CurrentMode == OPERATION_MODE_TX && (interrupt2Status & IRQ_2_PACKET_SENT) == IRQ_2_PACKET_SENT)
  {
    // Enter RX for ACK
    RFM69HCW_SetMode(OPERATION_MODE_RX);
  }
  // While explicitly in RX mode and the CRC OK flag is set, then this should be the ACK payload
  else if (CurrentMode == OPERATION_MODE_RX && (interrupt2Status & IRQ_2_CRC_OK) == IRQ_2_CRC_OK)
  {
    // Enter StandBy after ACK
    RFM69HCW_SetMode(OPERATION_MODE_STANDBY);

    // Read Metadata
    for (dataIdx = 0; dataIdx < MetadataLength; dataIdx++)
      RFM69HCW_ReadRegister(FIFO);

    // Check if the Payload matches the Last Sent ACK
    if (RFM69HCW_ReadRegister(FIFO) == LAST_SENT_ACK)
      ACK_STATUS = ACK_PAYLOAD_PASSED;

    // Confirm Packet was received
    ACK_STATUS |= ACK_PAYLOAD_RECEIVED;
  }
  // If the current mode is standby, and this interrupt was triggered, this was a PayloadReady event
  // in it's periodic transitions to the RX state
  else if (CurrentMode == OPERATION_MODE_STANDBY && (interrupt2Status & IRQ_2_PAYLOAD_READY) == IRQ_2_PAYLOAD_READY)
  {
    HasNewData = true;

    // Read Metadata (Payload Length and Sender ID rn)
    for (dataIdx = 0; dataIdx < MetadataLength; dataIdx++)
      RX_Data_Metadata[dataIdx] = RFM69HCW_ReadRegister(FIFO);

    // Remove Metadata Length from Total Payload Length
    RX_Data_Metadata[0] -= MetadataLength;

    // Read Data Stream
    for (dataIdx = 0; dataIdx < RX_Data_Metadata[0]; dataIdx++)
      RX_Data_Buffer[dataIdx] = RFM69HCW_ReadRegister(FIFO);

    // Null at the end of data
    RX_Data_Buffer[RX_Data_Metadata[0]] = 0;

    // Send the ACK byte received as ACK
    RFM69HCW_WriteRegister(FIFO, RX_Data_Metadata[2]);
    RFM69HCW_SetMode(OPERATION_MODE_TX);
    while ((RFM69HCW_ReadRegister(IRQ_FLAGS_2) & IRQ_2_PACKET_SENT) != IRQ_2_PACKET_SENT)
      ;

    // Go back to StandBy Mode
    RFM69HCW_SetMode(OPERATION_MODE_STANDBY);
  }
  else
    // Prevent Interrupt from being cleared
    Handled = false;

    // Clear Interrupt
  if (Handled)
    GPIO_PORTB_ICR_R |= DIO_0_INT_BIT;
}

static void RFM69HCW_Interrupt_Init(void)
{
  // Enable Port B clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;

  // Disable Alternate Functions on interrupt pins
  GPIO_PORTB_AFSEL_R &= ~DIO_0_INT_BIT;

  // Disable Peripheral functions on the interrupt pins
  GPIO_PORTB_PCTL_R &= ~INT_PCTL_M;

  // Configure interrupt pins as inputs
  GPIO_PORTB_DIR_R &= ~DIO_0_INT_BIT;

  // Enable Digital Mode on interrupt pins
  GPIO_PORTB_DEN_R |= DIO_0_INT_BIT;

  // Disable Analog Mode on interrupt pins
  GPIO_PORTB_AMSEL_R &= ~DIO_0_INT_BIT;

  // Disable interrupt mask on interrupt pins
  GPIO_PORTB_IM_R &= ~DIO_0_INT_BIT;

  // Configure for Edge-Detect interrupts
  GPIO_PORTB_IS_R &= ~DIO_0_INT_BIT;

  // Only listen on one edge event on the pin
  GPIO_PORTB_IBE_R &= ~DIO_0_INT_BIT;

  // Trigger interrupt on rising edge
  GPIO_PORTB_IEV_R |= DIO_0_INT_BIT;

  // Enable Port B's Interrupt Handler
  NVIC_EN0_R |= NVIC_EN0_INT1;

  // Configure Port B's priority
  NVIC_PRI0_R = (NVIC_PRI0_R & (unsigned)~NVIC_PRI0_INT1_M) | (RFM69HCW_INT_PRIORITY << NVIC_PRI0_INT1_S);

  // Clear the pins interrupt
  GPIO_PORTB_ICR_R |= DIO_0_INT_BIT;

  // Allow Interrupts on the pins to be detected
  GPIO_PORTB_IM_R |= DIO_0_INT_BIT;
}

static void RFM69HCW_Config(uint32_t bitRate, uint32_t deviation, uint8_t rxBW, uint8_t interPacketRxDelay)
{
  uint16_t idx = 0;
  uint32_t carrierFrequency = 0;

  // Ensure deviation is more than half the bit-rate.
  if (deviation < (bitRate / 2))
    while (1)
      ;

  do
  {
    deviceVersion = RFM69HCW_ReadRegister(VERSION);
  } while (deviceVersion != 0x24);

  // Put device in standby mode
  RFM69HCW_SetMode(OPERATION_MODE_STANDBY);

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

  // Enable sync word verification, with 3 words and no tolerance for errors
  RFM69HCW_WriteRegister(SYNC_CONFIG, SYNC_WORD_VERIFICATION | SYNC_WORD_BYTE_COUNT_3 | SYNC_WORD_NO_TOLERANCE);

  // Verify the Bit Sync value (0xAA or 0x55), Device Version, Network ID
  RFM69HCW_WriteRegister(SYNC_VALUE_1, 0xAA);
  RFM69HCW_WriteRegister(SYNC_VALUE_2, deviceVersion);
  RFM69HCW_WriteRegister(SYNC_VALUE_3, NETWORK_ID);

  // Configure the Packets to be of Variable Length, with Variable lengths and the addresses to be filtered for this node
  RFM69HCW_WriteRegister(PACKET_CONFIG_1, (PACKET_VARIABLE_LENGTH | PACKET_CRC_ENABLE | PACKET_ADDRESS_FILTER_NODE) & ~PACKET_CRC_AUTO_CLEAR_OFF);
  RFM69HCW_WriteRegister(PACKET_NODE_ADDR, MY_NODE_ID);

  // Trigger transmit start on non-empty FIFO buffer
  RFM69HCW_WriteRegister(FIFO_THRESHOLD, FIFO_TX_ON_NOT_EMPTY);

  // Set (max) payload length to 64
  RFM69HCW_WriteRegister(PAYLOAD_LENGTH, PAYLOAD_LENGTH_64);

  // Specify DC Offset to 4% of BW
  RFM69HCW_WriteRegister(RX_BANDWIDTH, RECEIVER_DC_OFFSET_CUTOFF_FREQ | rxBW);

  // Set Ramp-Time to 40us
  RFM69HCW_WriteRegister(PA_RAMP_TIME, PA_FSK_RAMP_TIME_40u);

  // Enable PA1 and use half of the max power (13dBm)
  RFM69HCW_WriteRegister(PA_LEVEL, PA1_ON | (PA_MAX_POWER / 2));

  // Enable Over Current Protection (required for high power)
  RFM69HCW_WriteRegister(CURRENT_PROTECTION, CURRENT_PROTECTION_ON);

  // Enable Automatic ACK after TX
  // RFM69HCW_WriteRegister(AUTO_MODES, AUTO_MODES_AUTO_RX_ACK);

  // Configure Listen Mode to stay in RX Mode, and only accept packets that match both the RSSI Threshold and Address
  RFM69HCW_WriteRegister(LISTEN_1, LISTEN_END_NO_ACTION | LISTEN_CRITERIA_THRESHOLD_ADDRESS | LISTEN_RESO_IRX_DEFAULT | LISTEN_RESO_IDLE_DEFAULT);

  // Set RSSI Threshold
  RFM69HCW_WriteRegister(RSSI_THRESHOLD, RSSI_THRESHOLD_DEFAULT);

  // Enable AES encryption, automatic RX phase restart and specified Inter Packet RX Delay
  RFM69HCW_WriteRegister(PACKET_CONFIG_2, (uint8_t)(PACKET_AES_ENCRYPTION | PACKET_AUTO_RX_RESTART | (unsigned)(interPacketRxDelay << PACKET_INTER_RX_DELAY_S)));

  // Set Cipher Key
  for (idx = 0; idx < AES_KEY_LAST - AES_KEY_FIRST; idx++)
    RFM69HCW_WriteRegister((ADDRESS)(AES_KEY_FIRST + idx), AES_CIPHER_KEY[idx]);

  // Disable Clock output
  RFM69HCW_WriteRegister(DIO_MAPPING_2, DIO_CLK_OUT_OFF);
}

void RFM69HCW_Init(uint32_t SYS_CLK, uint32_t SSI_CLK)
{
  // Initialize SysTick
  SysTick_Init();

  // Initialize the SPI pins
  SPI2_Init(SYS_CLK, SSI_CLK, SSI_CR0_FRF_MOTO, SSI_CR0_DSS_16);

  // Configure Wireless settings
  RFM69HCW_Config(
      25e3 /* 25kHz Bit-rate */,
      32e3 /* Deviation */,
      RECEIVER_BW_MANT_20 | 5 /* Set RxBwExp to 5, and use 20 for RxBwMant */,
      0 /* For 2^0 = 1 bit delay, to get 40us Ramp */
  );

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

  SPI2_StartTransmission();
  SPI2_Write(&byte, 1);
  SPI2_EndTransmission();
}

static void RFM69HCW_SetMode(MODE newMode)
{
  uint8_t modeSettings = 0;

  if (CurrentMode == newMode)
    return;

  // Get current operation mode settings, mask out current mode and enable automatic sequencing
  modeSettings = (RFM69HCW_ReadRegister(OPERATION_MODE) & ~(OPERATION_MODE_M | OPERATION_SEQUENCER_OFF));

  // Have to abort Listen Mode to change Operation mode
  if (modeSettings & OPERATION_LISTEN_ON)
  {
    // First stage
    modeSettings = (modeSettings | OPERATION_LISTEN_ABORT | newMode) & ~OPERATION_LISTEN_ON;
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);

    // Second stage
    modeSettings &= ~OPERATION_LISTEN_ABORT;
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);
  }
  else
  {
    modeSettings |= newMode;

    // Set new Mode
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings);
  }

  if (newMode == OPERATION_MODE_STANDBY)
    RFM69HCW_WriteRegister(OPERATION_MODE, modeSettings | OPERATION_LISTEN_ON);

  // Wait for MODE_READY
  while ((RFM69HCW_ReadRegister(IRQ_FLAGS_1) & IRQ_1_MODE_READY) != IRQ_1_MODE_READY)
    ;

  CurrentMode = newMode;
}

void RFM69HCW_SendPacket(uint8_t *data, uint8_t length)
{
  uint8_t dataIdx = 0;
  uint16_t TX_Metadata[((MetadataLength - 1) / 2) + 1] = {
      // All bytes after address & payload length
      WRITE(FIFO, length + MetadataLength - 1),
      (PEER_NODE_ID << 8) | ++LAST_SENT_ACK,
  };
  uint16_t TX_Payload[PAYLOAD_LENGTH_64 / 2] = {0};

  // Increment by 2 since we're taking two bytes out the data
  for (dataIdx = 0; dataIdx < length; dataIdx += 2)
  {
    // Merge the bytes together to form 16 bit words.
    TX_Payload[dataIdx] = (uint16_t)((data[dataIdx] << 8) | (((dataIdx + 1) < length) ? data[dataIdx + 1] : 0));
  }

  do
  {
    RFM69HCW_SetMode(OPERATION_MODE_STANDBY);

    while ((RFM69HCW_ReadRegister(IRQ_FLAGS_1) & IRQ_1_MODE_READY) != IRQ_1_MODE_READY)
      ;

    SPI2_StartTransmission();
    SPI2_Write(TX_Metadata, sizeof(TX_Metadata) / sizeof(TX_Metadata[0]));
    SPI2_Write(TX_Payload, length);
    SPI2_EndTransmission();

    // Enable Packet Sent/CRC OK Interrupt on DIO0
    RFM69HCW_WriteRegister(DIO_MAPPING_1, DIO_0_MAPPING_00);

    // Start sending Packet
    RFM69HCW_SetMode(OPERATION_MODE_TX);

    // Wait for ACK confirmation in Interrupt Handler
    ACK_STATUS = ACK_RESET;
    while ((ACK_STATUS & ACK_PAYLOAD_RECEIVED) != ACK_PAYLOAD_RECEIVED)
      ;
  }
  // Restart Transmission if the ACK was received but didn't match
  while ((ACK_STATUS & ACK_PAYLOAD_PASSED) != ACK_PAYLOAD_PASSED);
}
