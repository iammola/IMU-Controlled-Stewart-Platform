/*
Ensure "PART_TM4C123GH6PM TARGET_IS_BLIZZARD_RA1" is configured in Keil
In the Target Options, In the C/C++ tab, and in the Define field of the Preprocessor Symbols

Without it, several variables are not set which hinder the build of many of the
platform files (spi, uart) being successful, and it doesn't work by manually using the #define macro

I discovered this after searching the whole project and discovering the variables existed but weren't executed
because of an #ifndef condition, I used other TM4C123 examples from the Peripheral SDK and search for the condition variables
and found them in the *.uvproj files. That is defined in Keil. I couldn't find any documentation in the SDK talking about this
and assume it's an advantage of using CCS/Eclipse to build the project

"_USE_CLI_" is also required.
*/

#include "CC3100_P2P/P2P.h"

#include "PLL/PLL.h"

/*
 * Application's entry point
 */
int main(void)
{
  _i32 retVal = -1;

  PLL_Init();

  retVal = P2P_Init();
  if (retVal < 0)
  {
    CLI_Write(" Failed to initialize device \n\r");
    LOOP_FOREVER();
  }

  /*After calling this function, you can start sending data to CC3100 IP address on PORT_NUM */
  // retVal = UDP_StartServer(PORT_NUM);
  // if (retVal < 0)
  //   CLI_Write(" Failed to start UDP  server \n\r");
  // else
  //   CLI_Write(" UDP  client connected successfully \n\r");

  /* Stop the CC3100 device */
  retVal = sl_Stop(SL_STOP_TIMEOUT);
  if (retVal < 0)
  {
    CLI_Write(" Failed to stop device \n\r");
    LOOP_FOREVER();
  }

  return 0;
}
