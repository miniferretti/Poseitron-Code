#include "CAN_Alternate.hh"
#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <vector>
using namespace std;

void CAN0configure(int baud)
{
  if (baud == 10000)
    system("sudo ip link set can0 up type can bitrate 10000");
  else if (baud == 20000)
    system("sudo ip link set can0 up type can bitrate 20000");
  else if (baud == 50000)
    system("sudo ip link set can0 up type can bitrate 50000");
  else if (baud == 100000)
    system("sudo ip link set can0 up type can bitrate 100000");
  else if (baud == 125000)
    system("sudo ip link set can0 up type can bitrate 125000");
  else if (baud == 500000)
    system("sudo ip link set can0 up type can bitrate 500000");
  else if (baud == 250000)
    system("sudo ip link set can0 up type can bitrate 250000");
  else if (baud == 800000)
    system("sudo ip link set can0 up type can bitrate 800000");
  else if (baud == 1000000)
    system("sudo ip link set can0 up type can bitrate 1000000");
  else
    system("sudo ip link set can0 up type can bitrate 500000");
}

void CAN0pushPropDC(int dcG, int dcD)
{
  uint8_t dcGc = 128 * dcG / 100.0 + 128;
  uint8_t dcDc = 128 * dcD / 100.0 + 128;
  dcGc = dcGc >> 2;
  dcDc = dcDc >> 2;

  system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#25FF" + hexStr2(&dcGc, 1)).c_str());
  system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#26FF" + hexStr2(&dcDc, 1)).c_str());
  printf(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#25FF" + hexStr2(&dcGc, 2)).c_str());
  printf("\r\n");
  printf(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#26FF" + hexStr2(&dcDc, 2)).c_str());
  printf("\r\n");
}

void CAN0ctrl_motor(int state)
{
  if (state)
  {
    system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#1E3000").c_str());
  }
  else
  {
    system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#1E30FF").c_str());
  }
}

string uint8_to_hex_string(const uint8_t *v, const size_t s)
{
  stringstream ss;

  ss << setfill('0');

  ss << hex << setw(2) << v << endl;

  printf((ss.str()).c_str());
  printf("\r\n");

  return (ss.str());
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

void CAN0ctrl_led(int state)
{
  if (state)
  {
    system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#1E4040").c_str());
  }
  else
  {
    system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#1E4000").c_str());
  }
}

string int_to_hex_string(int theInt)
{
  stringstream ss;
  ss << hex << theInt;
  return ss.str();
}