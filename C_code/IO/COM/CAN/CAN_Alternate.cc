#include "CAN_Alternate.hh"
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
  rfilter.can_mask = 0x1FFFFFF0;

  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

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
}

void CAN0_Alternate::CAN0pushPropDC(int dcG, int dcD)
{
  uint8_t dcGc = 128 * dcG / 100.0 + 128;
  uint8_t dcDc = 128 * dcD / 100.0 + 128;
  dcGc = dcGc >> 2;
  dcDc = dcDc >> 2;
  can_frame msg;
  can_frame msg1;

  // system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#25FF" + int_to_hex(dcGc)).c_str());
  //  system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#26FF" + int_to_hex(dcDc)).c_str());
  //printf(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#25FF" + int_to_hex(dcGc)).c_str());
  //printf("\r\n");
  //printf(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#26FF" + int_to_hex(dcDc)).c_str());
  //printf("\r\n");

  msg.can_id = CAN_MOT;
  msg.can_dlc = 3;
  msg.data[0] = 0x26;
  msg.data[1] = 0xFF;
  msg.data[2] = dcGc;

  write(s, &msg, sizeof(msg));
  usleep(DELAY);

  msg1.can_id = CAN_MOT;
  msg1.can_dlc = 3;
  msg1.data[0] = 0x25;
  msg1.data[1] = 0xFF;
  msg1.data[2] = dcDc;

  write(s, &msg1, sizeof(msg1));
  usleep(DELAY);
}

void CAN0_Alternate::CAN0ctrl_motor(int state)
{
  can_frame msg;

  if (state)
  {
    //system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#1E3000").c_str());

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
    // system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#1E30FF").c_str());
    msg.can_id = CAN_MOT;
    msg.can_dlc = 3;
    msg.data[0] = 0x1E;
    msg.data[1] = 0x30;
    msg.data[2] = 0xFF;

    write(s, &msg, sizeof(msg));
    usleep(DELAY);
  }
}

/*string uint8_to_hex_string(const uint8_t *v, const size_t s)
{
  stringstream ss;

  ss << setfill('0');

  ss << hex << setw(2) << v << endl;

  printf((ss.str()).c_str());
  printf("\r\n");

  return (ss.str());
}*/

void CAN0_Alternate::CAN0ctrl_led(int state)
{

  can_frame msg;

  if (state)
  {
    //system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#1E4040").c_str());

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
    //system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#1E4000").c_str());

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

/*string int_to_hex(int a)
{
  string str = "";
  switch (a / 16)
  {
  case 0:
    str += "0";
    break;
  case 1:
    str += "1";
    break;
  case 2:
    str += "2";
    break;
  case 3:
    str += "3";
    break;
  case 4:
    str += "4";
    break;
  case 5:
    str += "5";
    break;
  case 6:
    str += "6";
    break;
  case 7:
    str += "7";
    break;
  case 8:
    str += "8";
    break;
  case 9:
    str += "9";
    break;
  case 10:
    str += "A";
    break;
  case 11:
    str += "B";
    break;
  case 12:
    str += "C";
    break;
  case 13:
    str += "D";
    break;
  case 14:
    str += "E";
    break;
  case 15:
    str += "F";
    break;
  }
  switch (a % 16)
  {
  case 0:
    str += "0";
    break;
  case 1:
    str += "1";
    break;
  case 2:
    str += "2";
    break;
  case 3:
    str += "3";
    break;
  case 4:
    str += "4";
    break;
  case 5:
    str += "5";
    break;
  case 6:
    str += "6";
    break;
  case 7:
    str += "7";
    break;
  case 8:
    str += "8";
    break;
  case 9:
    str += "9";
    break;
  case 10:
    str += "A";
    break;
  case 11:
    str += "B";
    break;
  case 12:
    str += "C";
    break;
  case 13:
    str += "D";
    break;
  case 14:
    str += "E";
    break;
  case 15:
    str += "F";
    break;
  }
  return str;
}

std::string hexStr2(unsigned char *data, int len)
{
  constexpr char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                             '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  std::string s(len * 2, ' ');
  for (int i = 0; i < len; ++i)
  {
    s[2 * i] = hexmap[(data[i] & 0xF0) >> 4];
    s[2 * i + 1] = hexmap[data[i] & 0x0F];
  }
  return s;
}

string int_to_hex_string(int theInt)
{
  stringstream ss;
  ss << hex << theInt;
  return ss.str();
}*/
// For fruther info on the routines visit: https://github.com/rhyttr/SocketCAN/blob/master/test/tst-raw.c