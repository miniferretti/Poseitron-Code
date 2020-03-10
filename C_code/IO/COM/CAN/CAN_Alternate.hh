#define CAN_MOT 0x508
#include <sstream>
#include <string>
#include <iostream>

 using namespace std;

void CAN0pushPropDC(int dcG,int dcD);
string uint8_to_hex_string(const uint8_t *v, const size_t s);
void CAN0ctrl_motor(int state);
void CAN0ctrl_led(int state);
string int_to_hex_string(int theInt);
void CAN0configure();