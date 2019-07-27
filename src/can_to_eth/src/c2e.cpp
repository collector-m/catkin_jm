/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <can_to_eth/c2e.h>
#include "console_bridge/console.h"

CAN_ETH::CAN_ETH() 
{
}

CAN_ETH::~CAN_ETH()
{
}

void CAN_ETH::connect(std::string host, int port)
{
  if (!connected_)
  {
    logDebug("Creating non-blocking socket.");
    socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_fd_)
    {

      struct sockaddr_in stSockAddr;
      stSockAddr.sin_family = PF_INET;
      stSockAddr.sin_port = htons(port);
      inet_pton(AF_INET, host.c_str(), &stSockAddr.sin_addr);

      logDebug("Connecting socket to laser.");
      int ret = ::connect(socket_fd_, (struct sockaddr *) &stSockAddr, sizeof(stSockAddr));

      if (ret == 0)
      {
        connected_ = true;
        printf("Connection succeeded.\n");

        socklen_t sendbuflen = 0;
        socklen_t slen = sizeof(sendbuflen);
        getsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, (void*)& sendbuflen, &slen);
        printf("default sendbuf len: %d\n", sendbuflen);

        sendbuflen = 425984;
        setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, (void*)& sendbuflen, slen);
        getsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, (void*)& sendbuflen, &slen);
        printf("now sendbuf len: %d\n", sendbuflen);

        int flags;
        flags = fcntl(socket_fd_, F_GETFL, 0);
        printf("default block mode: %d\n", flags);
        flags = fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
        printf("current block mode: %d\n", flags);
      }
    }
  }
}

void CAN_ETH::disconnect()
{
  if (connected_)
  {
    close(socket_fd_);
    connected_ = false;
  }
}

bool CAN_ETH::isConnected()
{
  return connected_;
}

void CAN_ETH::sent_to_can()
{
  unsigned char CanBuffer[650];
  CAN_MESSAGE temp_canmessge;
  uint message_num=0,byte_num=0;
  for(;message_num<50;message_num++)
  {
    if(!can_Tmessage.empty())
    {
        temp_canmessge = can_Tmessage.front();
        memcpy(&CanBuffer[byte_num],temp_canmessge.can_data,13);
        can_Tmessage.erase(can_Tmessage.begin());
        byte_num += 13;
    }  
  }
  write(socket_fd_, CanBuffer, byte_num);
}

void CAN_ETH::receive_from_can()
{
  unsigned char CanBuffer[650];
  CAN_MESSAGE temp_canmessge;
  uint message_num=0,byte_num=0;
  int len;
  len = read(socket_fd_, CanBuffer, 650);
  if(len>0)
  {
    message_num = len/13 ;
    for(int i=0;i<message_num;i++)
    {
      memcpy(temp_canmessge.can_data,&CanBuffer[byte_num],13);
      can_Rmessage.push_back(temp_canmessge);
      byte_num += 13;
    }
  }
}



/*
void LMS1xx::startMeas()
{
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sMN LMCstartmeas", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);
}

void LMS1xx::stopMeas()
{
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sMN LMCstopmeas", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);
}
*/
#if 0
status_t CAN_TO_ETH::queryStatus()
{
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sRN STlms", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);

  int ret;
  sscanf((buf + 10), "%d", &ret);

  return (status_t) ret;
}

void CAN_TO_ETH::login()
{
  char buf[100];
  int result;
  sprintf(buf, "%c%s%c", 0x02, "sMN SetAccessMode 03 F4724744", 0x03);

  fd_set readset;
  struct timeval timeout;


  do   //loop until data is available to read
  {
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    write(socket_fd_, buf, strlen(buf));

    FD_ZERO(&readset);
    FD_SET(socket_fd_, &readset);
    result = select(socket_fd_ + 1, &readset, NULL, NULL, &timeout);

  }
  while (result <= 0);

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);
}

scanCfg LMS1xx::getScanCfg() const
{
  scanCfg cfg;
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sRN LMPscancfg", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);

  sscanf(buf + 1, "%*s %*s %X %*d %X %X %X", &cfg.scaningFrequency,
         &cfg.angleResolution, &cfg.startAngle, &cfg.stopAngle);
  return cfg;
}



  char buf[100];
  sprintf(buf, "%c%s %X +1 %X %X %X%c", 0x02, "sMN mLMPsetscancfg",
          cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle,
          cfg.stopAngle, 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  buf[len - 1] = 0;
}

void LMS1xx::setScanDataCfg(const scanDataCfg &cfg)
{
  char buf[100];
  sprintf(buf, "%c%s %02X 00 %d %d 0 %02X 00 %d %d 0 %d +%d%c", 0x02,
          "sWN LMDscandatacfg", cfg.outputChannel, cfg.remission ? 1 : 0,
          cfg.resolution, cfg.encoder, cfg.position ? 1 : 0,
          cfg.deviceName ? 1 : 0, cfg.timestamp ? 1 : 0, cfg.outputInterval, 0x03);
  logDebug("TX: %s", buf);
  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  buf[len - 1] = 0;
}

scanOutputRange LMS1xx::getScanOutputRange() const
{
  scanOutputRange outputRange;
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sRN LMPoutputRange", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  sscanf(buf + 1, "%*s %*s %*d %X %X %X", &outputRange.angleResolution,
         &outputRange.startAngle, &outputRange.stopAngle);
  return outputRange;
}

void LMS1xx::scanContinous(int start)
{
  char buf[100];
  sprintf(buf, "%c%s %d%c", 0x02, "sEN LMDscandata", start, 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  if (buf[0] != 0x02)
    logError("invalid packet recieved");

  buf[len] = 0;
  logDebug("RX: %s", buf);
}

bool LMS1xx::getScanData(scanData* scan_data)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(socket_fd_, &rfds);

  // Block a total of up to 100ms waiting for more data from the laser.
  while (1)
  {
    // Would be great to depend on linux's behaviour of updating the timeval, but unfortunately
    // that's non-POSIX (doesn't work on OS X, for example).
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    logDebug("entering select()", tv.tv_usec);
    int retval = select(socket_fd_ + 1, &rfds, NULL, NULL, &tv);
    logDebug("returned %d from select()", retval);
    if (retval)
    {

      int ret = read(socket_fd_, buffer_ + total_length_, sizeof(buffer_) - total_length_);
     /*
      buffer_.readFrom(socket_fd_);

      // Will return pointer if a complete message exists in the buffer,
      // otherwise will return null.
      char* buffer_data = buffer_.getNextBuffer();

      if (buffer_data)
      {
        parseScanData(buffer_data, scan_data);
        buffer_.popLastBuffer();
        return true;
      }
      */
    }
    else
    {
      // Select timed out or there was an fd error.
      return false;
    }
  }
}


void LMS1xx::parseScanData(char* buffer, scanData* data)
{

}

void LMS1xx::saveConfig()
{
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sMN mEEwriteall", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);
}

void LMS1xx::startDevice()
{
  char buf[100];
  sprintf(buf, "%c%s%c", 0x02, "sMN Run", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  if (buf[0] != 0x02)
    logWarn("invalid packet recieved");
  buf[len] = 0;
  logDebug("RX: %s", buf);
}
#endif