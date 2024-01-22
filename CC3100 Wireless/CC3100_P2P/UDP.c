#include <stdint.h>

#include "sl_common.h"
#include "simplelink.h"

#define BUF_SIZE 1400
#define NO_OF_PACKETS 1000

static union
{
  uint8_t BsdBuf[BUF_SIZE];
  uint32_t demobuf[BUF_SIZE / 4];
} uBuf;

/*!
    \brief Opening a UDP client side socket and sending data

    This function opens a UDP socket and tries to send data to a UDP server
    IP_ADDR waiting on port PORT_NUM.
    Then the function will send 1000 UDP packets to the server.

    \param[in]      port number on which the server will be listening on

    \return         0 on success, -1 on Error.

    \note

    \warning
*/
int32_t UDP_SendData(uint16_t Port)
{
  SlSockAddrIn_t Addr;
  uint16_t idx = 0;
  uint16_t AddrSize = 0;
  int16_t SockID = 0;
  int16_t Status = 0;
  uint32_t LoopCount = 0;

  for (idx = 0; idx < BUF_SIZE; idx++)
  {
    uBuf.BsdBuf[idx] = (uint8_t)(idx % 10);
  }

  Addr.sin_family = SL_AF_INET;
  Addr.sin_port = sl_Htons((uint16_t)Port);
  Addr.sin_addr.s_addr = sl_Htonl(g_DeviceIp);

  AddrSize = sizeof(SlSockAddrIn_t);

  SockID = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
  if (SockID < 0)
  {
    CLI_Write((unsigned char *)" [UDP Client] Create socket Error \n\r");
    ASSERT_ON_ERROR(SockID)
  }

  while (LoopCount < NO_OF_PACKETS)
  {
    uBuf.BsdBuf[0] = LoopCount >> 24 & 0xFF;
    uBuf.BsdBuf[1] = LoopCount >> 16 & 0xFF;
    uBuf.BsdBuf[2] = LoopCount >> 8 & 0xFF;
    uBuf.BsdBuf[3] = LoopCount & 0xFF;

    Status = sl_SendTo(SockID, uBuf.BsdBuf, BUF_SIZE, 0, (SlSockAddr_t *)&Addr, AddrSize);
    if (Status <= 0)
    {
      Status = sl_Close(SockID);
      CLI_Write((unsigned char *)" [UDP Client] Data send Error \n\r");
      ASSERT_ON_ERROR(UDP_SEND_ERROR)
    }

    LoopCount++;
  }

  Status = sl_Close(SockID);
  ASSERT_ON_ERROR(Status)

  return SUCCESS;
}

/*!
    \brief Opening a UDP server side socket and receiving data

    This function opens a UDP socket in Listen mode and waits for incoming
    UDP packets from the connected client.

    \param[in]      port number on which the server will be listening on

    \return         0 on success, Negative value on Error.

    \note

    \warning
*/
int32_t UDP_ReadData(uint16_t Port)
{
  SlSockAddrIn_t Addr;
  SlSockAddrIn_t LocalAddr;
  uint16_t idx = 0;
  uint16_t AddrSize = 0;
  int16_t SockID = 0;
  int16_t Status = 0;
  uint16_t LoopCount = 0;
  uint16_t recvSize = 0;

  for (idx = 0; idx < BUF_SIZE; idx++)
  {
    uBuf.BsdBuf[idx] = (uint8_t)(idx % 10);
  }

  LocalAddr.sin_family = SL_AF_INET;
  LocalAddr.sin_port = sl_Htons((uint16_t)Port);
  LocalAddr.sin_addr.s_addr = 0;

  SockID = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
  if (SockID < 0)
  {
    CLI_Write((unsigned char *)" [UDP Server] Create socket Error \n\r");
    ASSERT_ON_ERROR(SockID)
  }

  AddrSize = sizeof(SlSockAddrIn_t);
  Status = sl_Bind(SockID, (SlSockAddr_t *)&LocalAddr, AddrSize);
  if (Status < 0)
  {
    Status = sl_Close(SockID);
    CLI_Write((unsigned char *)" [UDP Server] Socket address assignment Error \n\r");
    ASSERT_ON_ERROR(Status)
  }

  while (LoopCount < NO_OF_PACKETS)
  {
    recvSize = BUF_SIZE;

    do
    {
      Status = sl_RecvFrom(SockID, &uBuf.BsdBuf[BUF_SIZE - recvSize], recvSize, 0,
                           (SlSockAddr_t *)&Addr, (SlSocklen_t *)&AddrSize);
      if (Status < 0)
      {
        sl_Close(SockID);
        CLI_Write((unsigned char *)" [UDP Server] Data recv Error \n\r");
        ASSERT_ON_ERROR(UDP_RECV_ERROR)
      }

      recvSize -= Status;

    } while (recvSize > 0);

    LoopCount++;
  }

  Status = sl_Close(SockID);
  ASSERT_ON_ERROR(Status)

  return SUCCESS;
}
