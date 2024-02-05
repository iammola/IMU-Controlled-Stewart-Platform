#include <stdint.h>
#include <stdbool.h>

#include "SysTick.h"
#include "tm4c123gh6pm.h"

#include "SPI/SPI.h"
#include "RFM69HCW.h"

#define DIO_0_INT_BIT 3 // PB3

#define INT_PCTL_M GPIO_PCTL_PB2_M | GPIO_PCTL_PB3_M

#define READ(addr) (uint8_t)(0x80 | addr)
#define WRITE(addr) (uint8_t)(0x7F & addr)

#define NETWORK_ID 23
const char AES_CIPHER_KEY[16] = "($HJ#BUCA823nGU1";

#ifdef __DEVICE_1__
#define MY_NODE_ID 54
#define PEER_NODE_ID 92
#else
#define MY_NODE_ID 92
#define PEER_NODE_ID 54
#endif

static void RFM69HCW_Interrupt_Init(void);
static void RFM69HCW_Config(uint32_t bitRate, uint32_t deviation, uint8_t rxBW, uint8_t interPacketRxDelay);
static uint8_t RFM69HCW_ReadRegister(ADDRESS REGISTER);
static void RFM69HCW_WriteRegister(ADDRESS REGISTER, uint8_t *data, uint8_t length);
static void RFM69HCW_WriteRegisterByte(ADDRESS REGISTER, uint8_t data);
static void RFM69HCW_SetMode(MODE newMode);

static uint8_t LAST_SENT_ACK = 0;
static uint8_t deviceVersion = 0;
static uint8_t ACK_STATUS = ACK_RESET;

bool HasNewData = false;
uint8_t RX_Data_Metadata[MetadataLength] = {0};
uint8_t RX_Data_Buffer[PAYLOAD_LENGTH_64 + 1] = {0};

MODE CurrentMode;

void GPIOB_Handler(void)
{
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
    RFM69HCW_WriteRegisterByte(FIFO, RX_Data_Metadata[2]);
    RFM69HCW_SetMode(OPERATION_MODE_TX);
    while ((RFM69HCW_ReadRegister(IRQ_FLAGS_2) & IRQ_2_PACKET_SENT) != IRQ_2_PACKET_SENT)
      ;

    // Go back to StandBy Mode
    RFM69HCW_SetMode(OPERATION_MODE_STANDBY);
  }

  GPIO_PORTB_MIS_R |= DIO_0_INT_BIT;
}

static void RFM69HCW_Interrupt_Init(void)
{
  // Enable Port D clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;

  // Disable Alternate Functions on interrupt pins
  GPIO_PORTD_AFSEL_R &= ~DIO_0_INT_BIT;

  // Disable Peripheral functions on the interrupt pins
  GPIO_PORTD_PCTL_R &= ~INT_PCTL_M;

  // Configure interrupt pins as inputs
  GPIO_PORTD_DIR_R &= ~DIO_0_INT_BIT;

  // Enable Digital Mode on interrupt pins
  GPIO_PORTD_DEN_R |= DIO_0_INT_BIT;

  // Disable Analog Mode on interrupt pins
  GPIO_PORTD_AMSEL_R &= ~DIO_0_INT_BIT;

  // Disable interrupt mask on interrupt pins
  GPIO_PORTD_IM_R &= ~(DIO_0_INT_BIT);

  // Configure for Edge-Detect interrupts
  GPIO_PORTD_IS_R &= ~DIO_0_INT_BIT;

  // Only listen on one edge event on the pin
  GPIO_PORTD_IBE_R &= ~DIO_0_INT_BIT;

  // Trigger interrupt on rising edge
  GPIO_PORTD_IEV_R |= DIO_0_INT_BIT;

  // Enable Port B's Interrupt Handler
  NVIC_EN0_R |= NVIC_EN0_INT1;

  // Configure Port B's priority
  NVIC_PRI0_R = (NVIC_PRI0_R & ~NVIC_PRI0_INT1_M) | (RFM69HCW_INT_PRIORITY << NVIC_PRI0_INT1_S);

  // Clear the pins interrupt
  GPIO_PORTD_ICR_R |= DIO_0_INT_BIT;

  // Allow Interrupts on the pins to be detected
  GPIO_PORTD_IM_R |= DIO_0_INT_BIT;
}

static void RFM69HCW_Config(uint32_t bitRate, uint32_t deviation, uint8_t rxBW, uint8_t interPacketRxDelay)
{
  uint16_t idx = 0;
  uint32_t carrierFrequency = 0;

  // Ensure deviation is more than half the bit-rate.
  if (deviation < (bitRate / 2))
    while (1)
      ;

  deviceVersion = RFM69HCW_ReadRegister(VERSION);

  // Put device in standby mode and turn on listening mode and automatic sequencing
  RFM69HCW_WriteRegisterByte(OPERATION_MODE, (OPERATION_MODE_STANDBY | OPERATION_LISTEN_ON) & ~(OPERATION_SEQUENCER_OFF));

  // Use FSK modulation, packet data mode and no modulation shaping
  RFM69HCW_WriteRegisterByte(DATA_MODULATION, DATA_MODULATION_NO_SHAPING | DATA_MODULATION_FSK | DATA_MODULATION_MODE);

  // Set Data Bit-Rate
  bitRate = F_XOSC / bitRate;
  RFM69HCW_WriteRegisterByte(BITRATE_FIRST_BYTE, bitRate >> 8);
  RFM69HCW_WriteRegisterByte(BITRATE_LAST_BYTE, bitRate & 0xFF);

  // Set Frequency Deviation
  deviation = bitRate / (2 * F_STEP);
  RFM69HCW_WriteRegisterByte(DEVIATION_FIRST_BYTE, (deviation >> 8) & 0x3F);
  RFM69HCW_WriteRegisterByte(DEVIATION_LAST_BYTE, bitRate & 0xFF);

  // Set Carrier Frequency
  carrierFrequency = MODULE_FREQUENCY / F_STEP;
  RFM69HCW_WriteRegisterByte(CARRIER_FREQUENCY_FIRST_BYTE, carrierFrequency >> 16);
  RFM69HCW_WriteRegisterByte(CARRIER_FREQUENCY_MID_BYTE, (carrierFrequency & 0xFF00) >> 8);
  RFM69HCW_WriteRegisterByte(CARRIER_FREQUENCY_LAST_BYTE, carrierFrequency & 0xFF);

  // Enable sync word verification, with 3 words and no tolerance for errors
  RFM69HCW_WriteRegisterByte(SYNC_CONFIG, SYNC_WORD_VERIFICATION | SYNC_WORD_BYTE_COUNT_3 | SYNC_WORD_NO_TOLERANCE);

  // Verify the Bit Sync value (0xAA or 0x55), Device Version, Network ID
  RFM69HCW_WriteRegisterByte(SYNC_VALUE_1, 0xAA);
  RFM69HCW_WriteRegisterByte(SYNC_VALUE_2, deviceVersion);
  RFM69HCW_WriteRegisterByte(SYNC_VALUE_3, NETWORK_ID);

  // Configure the Packets to be of Variable Length, with Variable lengths and the addresses to be filtered for this node
  RFM69HCW_WriteRegisterByte(PACKET_CONFIG_1, (PACKET_VARIABLE_LENGTH | PACKET_CRC_ENABLE | PACKET_ADDRESS_FILTER_NODE) & ~PACKET_CRC_AUTO_CLEAR_OFF);
  RFM69HCW_WriteRegisterByte(PACKET_NODE_ADDR, MY_NODE_ID);

  // Trigger transmit start on non-empty FIFO buffer
  RFM69HCW_WriteRegisterByte(FIFO_THRESHOLD, FIFO_TX_ON_NOT_EMPTY);

  // Set (max) payload length to 64
  RFM69HCW_WriteRegisterByte(PAYLOAD_LENGTH, PAYLOAD_LENGTH_64);

  // Specify DC Offset to 4% of BW
  RFM69HCW_WriteRegisterByte(RX_BANDWIDTH, RECEIVER_DC_OFFSET_CUTOFF_FREQ | rxBW);

  // Set Ramp-Time to 40us
  RFM69HCW_WriteRegisterByte(PA_RAMP_TIME, PA_FSK_RAMP_TIME_40u);

  // Enable PA1 and use half of the max power (13dBm)
  RFM69HCW_WriteRegisterByte(PA_LEVEL, PA1_ON | (PA_MAX_POWER / 2));

  // Enable Over Current Protection (required for high power)
  RFM69HCW_WriteRegisterByte(CURRENT_PROTECTION, CURRENT_PROTECTION_ON);

  // Enable Automatic ACK after TX
  // RFM69HCW_WriteRegisterByte(AUTO_MODES, AUTO_MODES_AUTO_RX_ACK);

  // Configure Listen Mode to stay in RX Mode, and only accept packets that match both the RSSI Threshold and Address
  RFM69HCW_WriteRegisterByte(LISTEN_1, LISTEN_END_NO_ACTION | LISTEN_CRITERIA_THRESHOLD_ADDRESS | LISTEN_RESO_IRX_DEFAULT | LISTEN_RESO_IDLE_DEFAULT);

  // Configure Listen Mode to stay in RX Mode, and only accept packets that match both the RSSI Threshold and Address
  RFM69HCW_WriteRegisterByte(RSSI_THRESHOLD, RSSI_THRESHOLD_DEFAULT);

  // Enable AES encryption, automatic RX phase restart and specified Inter Packet RX Delay
  RFM69HCW_WriteRegisterByte(PACKET_CONFIG_2, PACKET_AES_ENCRYPTION | PACKET_AUTO_RX_RESTART | (interPacketRxDelay << PACKET_INTER_RX_DELAY_S));

  // Set Cipher Key
  for (idx = 0; idx < AES_KEY_LAST - AES_KEY_FIRST; idx++)
    RFM69HCW_WriteRegisterByte(AES_KEY_FIRST + idx, AES_CIPHER_KEY[idx]);

  // Disable Clock output
  RFM69HCW_WriteRegisterByte(DIO_MAPPING_2, DIO_CLK_OUT_OFF);
}

void RFM69HCW_Init(uint32_t SYS_CLK, uint32_t SSI_CLK)
{
  SysTick_Init();

  // Initialize the SPI pins
  SPI2_Init(SYS_CLK, SSI_CLK, SSI_CR0_FRF_MOTO, SSI_CR0_DSS_8);

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
  SPI2_Read(REGISTER, &response, 1);
  SPI2_EndTransmission();

  return response & 0xFF;
}

static void RFM69HCW_WriteRegister(ADDRESS REGISTER, uint8_t *data, uint8_t length)
{
  uint16_t byte = WRITE(REGISTER);

  SPI2_StartTransmission();
  SPI2_Write(&byte, 1);
  SPI2_Write((uint16_t *)data, length);
  SPI2_EndTransmission();
}

static void RFM69HCW_WriteRegisterByte(ADDRESS REGISTER, uint8_t data)
{
  RFM69HCW_WriteRegister(REGISTER, &data, 1);
}

static void RFM69HCW_SetMode(MODE newMode)
{
  uint8_t modeSettings = 0;

  if (CurrentMode == newMode)
    return;

  // Get current operation mode settings
  modeSettings = RFM69HCW_ReadRegister(OPERATION_MODE);

  // Mask out current mode
  modeSettings &= ~OPERATION_MODE_M;

  // Set new Mode
  RFM69HCW_WriteRegisterByte(OPERATION_MODE, modeSettings | newMode);

  CurrentMode = newMode;
}

void RFM69HCW_SendPacket(uint8_t *data, uint8_t length)
{
  uint16_t TX_Data_Bytes[MetadataLength + 1] = {
      WRITE(FIFO),
      length + 1,
      MY_NODE_ID,
      ++LAST_SENT_ACK,
  };

  do
  {
    RFM69HCW_SetMode(OPERATION_MODE_STANDBY);
    while ((RFM69HCW_ReadRegister(IRQ_FLAGS_1) & IRQ_1_MODE_READY) != IRQ_1_MODE_READY)
      ;

    SPI2_StartTransmission();
    SPI2_Write(TX_Data_Bytes, sizeof(TX_Data_Bytes) / sizeof(TX_Data_Bytes[0]));
    SPI2_Write(data, length);
    SPI2_EndTransmission();

    // Enable Packet Sent/CRC OK Interrupt on DIO0
    RFM69HCW_WriteRegisterByte(DIO_MAPPING_1, DIO_0_MAPPING_00);

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
