#include "IO/COM/CAN/CAN_Alternate.hh"
#include <sstream>
#include <string>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <vector>
#include "time.h"

//CAN interface headers
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <wiringPi.h>
#include <libsocketcan.h>
#include <unistd.h>

using namespace std;

CAN0_Alternate::CAN0_Alternate(int baud)
{
  BR = baud;
  if (BR == 10000)
    system("sudo ip link set can0 up type can bitrate 10000");
  else if (BR == 20000)
    system("sudo ip link set can0 up type can bitrate 20000");
  else if (BR == 50000)
    system("sudo ip link set can0 up type can bitrate 50000");
  else if (BR == 100000)
    system("sudo ip link set can0 up type can bitrate 100000");
  else if (BR == 125000)
    system("sudo ip link set can0 up type can bitrate 125000");
  else if (BR == 500000)
    system("sudo ip link set can0 up type can bitrate 500000");
  else if (BR == 250000)
    system("sudo ip link set can0 up type can bitrate 250000");
  else if (BR == 800000)
    system("sudo ip link set can0 up type can bitrate 800000");
  else if (BR == 1000000)
    system("sudo ip link set can0 up type can bitrate 1000000");
  else
    system("sudo ip link set can0 up type can bitrate 500000");

  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("socket");
  }

  rfilter.can_id = 0x700;
  rfilter.can_mask = 0x1FFFF000;

  tv.tv_sec = 0; // 3 milliseconds timout for recieving a message
  tv.tv_usec = 80000;

  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
  setsockopt(s, SOL_CAN_RAW, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));

  addr.can_family = AF_CAN;

  strcpy(ifr.ifr_name, "can0");
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
  {
    perror("SIOCGIFINDEX");
  }
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    perror("bind");
  }

  //fd_set_blocking(s, 1);
}

void CAN0_Alternate::CAN0pushPropDC(double dcG, double dcD)
{
  uint8_t dcGc = (uint8_t)(128.0 * dcG / 100.0 + 128.0);
  uint8_t dcDc = (uint8_t)(128.0 * dcD / 100.0 + 128.0);
  dcGc = dcGc >> 2;
  dcDc = dcDc >> 2;
  struct can_frame msg;
  struct can_frame msg1;

  msg.can_id = CAN_MOT;
  msg.can_dlc = 3;
  msg.data[0] = 0x26;
  msg.data[1] = 0xFF;
  msg.data[2] = dcGc;

  write(s, &msg, sizeof(msg));
  // usleep(DELAY);

  msg1.can_id = CAN_MOT;
  msg1.can_dlc = 3;
  msg1.data[0] = 0x25;
  msg1.data[1] = 0xFF;
  msg1.data[2] = dcDc;

  write(s, &msg1, sizeof(msg1));
  // usleep(DELAY);
}

void CAN0_Alternate::CAN0ctrl_motor(int state)
{
  struct can_frame msg;

  if (state)
  {

    msg.can_id = CAN_MOT;
    msg.can_dlc = 3;
    msg.data[0] = 0x1E;
    msg.data[1] = 0x30;
    msg.data[2] = 0x00;

    write(s, &msg, sizeof(msg));
    usleep(DELAY);
  }
  else
  {
    msg.can_id = CAN_MOT;
    msg.can_dlc = 3;
    msg.data[0] = 0x1E;
    msg.data[1] = 0x30;
    msg.data[2] = 0xFF;

    write(s, &msg, sizeof(msg));
    usleep(DELAY);
  }
}

void CAN0_Alternate::CAN0ctrl_led(int state)
{

  struct can_frame msg;

  if (state)
  {

    msg.can_id = CAN_MOT;
    msg.can_dlc = 3;
    msg.data[0] = 0x1E;
    msg.data[1] = 0x40;
    msg.data[2] = 0x40;

    write(s, &msg, sizeof(msg));
    usleep(DELAY);
  }
  else
  {

    msg.can_id = CAN_MOT;
    msg.can_dlc = 3;
    msg.data[0] = 0x1E;
    msg.data[1] = 0x40;
    msg.data[2] = 0x00;

    write(s, &msg, sizeof(msg));
    usleep(DELAY);
  }
}

void CAN0_Alternate::CAN0close()
{
  close(s);
}

void CAN0_Alternate::msgClear(can_frame *fr)
{
  fr->can_id = 0;
  for (int i = 0; i < fr->can_dlc; i++)
  {
    fr->data[i] = 0;
  }
  fr->can_dlc = 0;
}

int CAN0_Alternate::getDistance(int dir, double *data)
{
  struct can_frame msg;
  struct can_frame msg2;
  int nbytes;

  if (dir)
  {

    msg.can_id = CAN_SENS_ARRAY_FRONT;
    msg.can_dlc = 3;
    msg.data[0] = 0x00;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;

    write(s, &msg, sizeof(msg));
    // usleep(DELAY);
  }
  else
  {

    msg.can_id = CAN_SENS_ARRAY_BACK;
    msg.can_dlc = 3;
    msg.data[0] = 0x00;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;

    write(s, &msg, sizeof(msg));
    //  usleep(DELAY);
  }

  while (nbytes < 0)
  {
    nbytes = read(s, &msg2, sizeof(msg2));
  }

  if (nbytes < 0)
  {
    //  printf("Data not ready\r\n");
    return 0;
  }
  else
  {
    for (int i = 0; i < msg2.can_dlc; i++)
    {
      if (msg2.data[i] == 0)
      {
        data[i] = 255;
      }
      else
      {
        data[i] = (double)msg2.data[i];
      }
    }

    printf("%f %f %f %f %f\r\n", data[0], data[1], data[2], data[3], data[4]);
    return 1;
  }
}

// For fruther info on the routines visit: https://github.com/rhyttr/SocketCAN/blob/master/test/tst-raw.c

int CAN0_Alternate::fd_set_blocking(int fd, int blocking)
{
  /* Save the current flags */
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags == -1)
    return 0;

  if (blocking)
    flags &= ~O_NONBLOCK;
  else
    flags |= O_NONBLOCK;
  return fcntl(fd, F_SETFL, flags) != -1;
}