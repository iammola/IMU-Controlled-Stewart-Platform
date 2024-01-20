#include "sl_common.h"
#include "simplelink/include/simplelink.h"

#define APPLICATION_VERSION "1.3.0"

#define SL_STOP_TIMEOUT 0xFF

/* Application specific status/error codes */
typedef enum
{
  DEVICE_NOT_IN_STATION_MODE = -0x7D0, /* Choosing this number to avoid overlap w/ host-driver's error codes */
  P2P_CONNECTION_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,
  UDP_SEND_ERROR = P2P_CONNECTION_FAILED - 1,
  UDP_RECV_ERROR = UDP_SEND_ERROR - 1,

  STATUS_CODE_MAX = -0xBB8
} e_AppStatusCodes;

/* Port to be used  by server*/
#define PORT_NUM 5001

#define BUF_SIZE 1400

#define NO_OF_PACKETS 1000

_u8 g_Status = 0;
_u32 g_DeviceIp = 0;
_i8 g_p2p_dev[MAXIMAL_SSID_LENGTH + 1];

union
{
  _u8 BsdBuf[BUF_SIZE];
  _u32 demobuf[BUF_SIZE / 4];
} uBuf;
