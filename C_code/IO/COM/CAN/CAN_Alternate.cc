#include "CAN_Alternate.hh"
#include <sstream>
#include <string>
#include <iostream>
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
  stringstream ss1;
  stringstream ss2;

  uint8_t pushD = dcD >> 2;
  uint8_t pushG = dcG >> 2;

  system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#25FF" + uint8_to_hex_string(&pushD, 2)).c_str());
  system(("cansend can0 " + int_to_hex_string(CAN_MOT) + "#25FF" + uint8_to_hex_string(&pushG, 2)).c_str());
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

  ss << hex << setfill('0');

  for (int i = 0; i < s; i++)
  {
    ss << hex << setw(2) << static_cast<int>(v[i]);
  }

  return ss.str();
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