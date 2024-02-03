#include <stdint.h>

#include "tm4c123gh6pm.h"

#include "SPI.h"

#define SSI3_CLK_BIT 1 << 0 // (PD0) SSI3CLK
#define SSI3_FSS_BIT 1 << 1 // (PD1) SSI3FSS (Chip Select)
#define SSI3_RX_BIT 1 << 2  // (PD2) SSI3Rx
#define SSI3_TX_BIT 1 << 3  // (PD3) SSI3Tx

#define SSI3_PINS (uint8_t)(SSI3_CLK_BIT | SSI3_FSS_BIT | SSI3_RX_BIT | SSI3_TX_BIT)
#define SSI3_PCTL (uint32_t)(GPIO_PCTL_PD0_SSI3CLK | GPIO_PCTL_PD1_SSI3FSS | GPIO_PCTL_PD2_SSI3RX | GPIO_PCTL_PD3_SSI3TX)
#define SSI3_PCTL_MASK (uint32_t)(GPIO_PCTL_PD0_M | GPIO_PCTL_PD1_M | GPIO_PCTL_PD2_M | GPIO_PCTL_PD3_M)

#define SSI3_FSS_ADDR (*((volatile uint32_t *)(0x40007000 | (SSI3_FSS_BIT << 2))))

#define WAIT_FOR_TX_SPACE()                        \
    while ((SSI3_SR_R & SSI_SR_TNF) != SSI_SR_TNF) \
        ;
#define WAIT_FOR_RX_DATA()                         \
    while ((SSI3_SR_R & SSI_SR_RNE) != SSI_SR_RNE) \
        ;
#define WAIT_FOR_IDLE()                            \
    while ((SSI3_SR_R & SSI_SR_BSY) == SSI_SR_BSY) \
        ;

void SPI3_Init(uint32_t SYS_CLK, uint32_t SSI_CLK, uint8_t frameConfig, uint8_t dataSize)
{
    uint32_t ssiSCR = 0;
    uint32_t ssiCPSR = 0;
    uint32_t maxBitRate = SYS_CLK / SSI_CLK;

    // Enable Port D clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;

    // Enable SSI module 3 clock
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R3;

    // Enable Alternate Functions on all pins
    GPIO_PORTD_AFSEL_R |= SSI3_PINS;

    // Enable SSI module 3 peripheral functions
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & ~SSI3_PCTL_MASK) | SSI3_PCTL;

    // Configure CLK, TX, and FSS as outputs
    GPIO_PORTD_DIR_R = (GPIO_PORTD_DIR_R & ~SSI3_PINS) | SSI3_CLK_BIT | SSI3_TX_BIT | SSI3_FSS_BIT;

    // Enable Digital Mode on pins
    GPIO_PORTD_DEN_R |= SSI3_PINS;

    // Disable Analog Mode on pins
    GPIO_PORTD_AMSEL_R &= ~SSI3_PINS;

    // Enable Pull-Up on Clock pin so is driven high when idle
    if (frameConfig & SSI_CR0_SPO)
        GPIO_PORTD_PUR_R |= SSI3_CLK_BIT;
    // Enable Pull-Down on Clock pin so is driven low when idle
    else
        GPIO_PORTD_PDR_R |= SSI3_CLK_BIT;

    // Disable SSI
    SSI3_CR1_R &= (unsigned)~SSI_CR1_SSE;

    // Enable Master mode
    SSI3_CR0_R &= (unsigned)~SSI_CR1_MS;

    // Configure for system clock
    SSI3_CC_R = SSI_CC_CS_SYSPLL;

    // As a master, the system clock must be 2 times faster than the SSI_CLK and the
    // SSI_CLK cannot be more than 25MHz
    if ((SYS_CLK < (SSI_CLK * 2)) || SSI_CLK > 25e6)
    {
        while (1)
            ;
    }

    do
    {
        // Increment Pre-scale Divisor by 2
        ssiCPSR += 2;

        // Calculate new clock rate
        ssiSCR = (maxBitRate / ssiCPSR) - 1;
    } while (ssiSCR > 255);

    // The SSI Clk frequency needs to be increased
    if (ssiCPSR > 254)
    {
        while (1)
            ;
    }

    // Set the calculated pre-scale divisor
    SSI3_CPSR_R = ssiCPSR;

    // Set the calculated clock rate, and the specified frame config and data size
    SSI3_CR0_R = (ssiSCR << SSI_CR0_SCR_S) | (frameConfig & ~(SSI_CR0_SCR_M | SSI_CR0_DSS_M)) | (dataSize & SSI_CR0_DSS_M);

    // Enable SSI
    SSI3_CR1_R |= SSI_CR1_SSE;

    // Force FSS pin high
    SSI3_FSS_ADDR = SSI3_FSS_BIT;
}

void SPI3_Read(uint16_t initData, uint16_t *result, uint8_t length)
{
    uint32_t clearData = 0;

    // Force FSS pin low
    SSI3_FSS_ADDR = 0;

    // Clear RX FIFO
    while ((SSI3_SR_R & SSI_SR_RNE) == SSI_SR_RNE)
    {
        clearData = SSI3_DR_R;
    }

    WAIT_FOR_TX_SPACE()

    SSI3_DR_R = initData;

    // Test more without each one or the other and add limits to loops
    WAIT_FOR_IDLE()

    while (length-- > 0)
    {
        WAIT_FOR_RX_DATA()

        *result = (uint16_t)SSI3_DR_R;
        result++;
    }

    // Force FSS pin high
    SSI3_FSS_ADDR = SSI3_FSS_BIT;
}

void SPI3_Write(uint16_t *data, uint8_t length)
{
    SSI3_FSS_ADDR = 0;

    while (length-- > 0)
    {
        WAIT_FOR_TX_SPACE()
        SSI3_DR_R = *data;

        data++;
    }

    WAIT_FOR_IDLE()

    SSI3_FSS_ADDR = SSI3_FSS_BIT;
}
