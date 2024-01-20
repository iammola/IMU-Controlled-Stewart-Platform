#include "CC3100_P2P/P2P.h"
#include "CC3100_P2P/UDP.h"
#include "CC3100_P2P/Global.h"

/*
 * Application's entry point
 */
int main(int argc, char **argv)
{
  _i32 retVal = -1;

  /* Stop WDT and initialize the system-clock of the MCU
     These functions needs to be implemented in PAL */
  stopWDT();
  initClk();

  P2P_Init();

  // /*After calling this function, you can start sending data to CC3100 IP
  //  * address on PORT_NUM */
  // retVal = BsdTcpServer(PORT_NUM);
  // if (retVal < 0)
  //   CLI_Write(" Failed to start TCP server \n\r");
  // else
  //   CLI_Write(" TCP client connected successfully \n\r");

  /* Stop the CC3100 device */
  retVal = sl_Stop(SL_STOP_TIMEOUT);
  if (retVal < 0)
    LOOP_FOREVER();

  return 0;
}
