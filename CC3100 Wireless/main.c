#include "CC3100_UDP/UDP.h"

void UDP_Init(void)
{
    _i32 retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    /* Configure command line interface */
    CLI_Configure();

    displayBanner();

    /*
     * Following function configures the device to default state by cleaning
     * the persistent settings stored in NVMEM (viz. connection profiles &
     * policies, power policy etc)
     *
     * Applications may choose to skip this step if the developer is sure
     * that the device is in its default state at start of application
     *
     * Note that all profiles and persistent settings that were done on the
     * device will be lost
     */
    retVal = configureSimpleLinkToDefaultState();
    if (retVal < 0)
    {
        CLI_Write(" Failed to configure the device in its default state \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Device is configured in default state \n\r");

    /*
     * Assumption is that the device is configured in station mode already
     * and it is in its default state
     */
    /* Initializing the CC3100 device */
    retVal = sl_Start(0, 0, 0);

    if ((retVal < 0) || (ROLE_STA != retVal))
    {
        CLI_Write(" Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Device started as STATION \n\r");

    /* Connecting to WLAN AP - Set with static parameters defined at the top
       After this call we will be connected and have IP address */
    retVal = establishConnectionWithAP();
    if (retVal < 0)
    {
        CLI_Write(" Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }
}

void UDP_Send(void)
{
    _i32 retVal = BsdUdpClient(PORT_NUM);

    if (retVal < 0)
        CLI_Write(" Failed to send data to UDP sevrer\n\r");
    else
        CLI_Write(" successfully sent data to UDP server \n\r");
}

void UDP_Read(void)
{

    _i32 retVal = BsdUdpServer(PORT_NUM);

    if (retVal < 0)
        CLI_Write(" Failed to read data from the UDP client \n\r");
    else
        CLI_Write(" Successfully received data from UDP client \n\r");
}

/*
 * Application's entry point
 */
int main(int argc, char **argv)
{
    _i32 retVal = -1;

    UDP_Init();

    CLI_Write(" Connection established w/ AP and IP is acquired \n\r");

    CLI_Write(" Started sending data to UDP server \n\r");

    UDP_Send();

    CLI_Write(" Waiting for data from UDP client \n\r");

    /* Stop the CC3100 device */
    retVal = sl_Stop(SL_STOP_TIMEOUT);

    if (retVal < 0)
    {
        LOOP_FOREVER();
    }

    return 0;
}
