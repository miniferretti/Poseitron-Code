#include "CAN_Alternate.hh"
#include <sstream>
#include <string>
#include <iostream>
using namespace std;

void CAN0pushPropDC(int dcG, int dcD)
{
  uint8_t dcGc = 128 * dcG / 100.0 + 128;
  uint8_t dcDc = 128 * dcD / 100.0 + 128;
  stringstream ss1;
  stringstream ss2;

  const uint8_t pushD = dcD >> 2;
  const uint8_t pushG = dcG >> 2;

  system(("cansend can0 508#25FF" + uint8_to_hex_string(&pushD, 2)).c_str());
  system(("cansend can0 508#25FF" + uint8_to_hex_string(&pushG, 2)).c_str());
}

void CAN0ctrl_motor(int state)
{
  if (state)
  {
    system("cansend can0 508#1E3000");
  }
  else
  {
    system("cansend can0 508#1E30FF");
  }
}

string uint8_to_hex_string(const uint8_t *v, const size_t s)
{
  std::stringstream ss;

  ss << hex << setfill('0');

  for (int i = 0; i < s; i++)
  {
    ss << hex << setw(2) << static_cast<int>(v[i]);
  }

  return ss.str();
}