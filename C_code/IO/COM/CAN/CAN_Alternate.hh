
#define CAN_MOT 0x508
#define DELAY 0
#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <vector>

//CAN header files
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <wiringPi.h>
#include <libsocketcan.h>
#include <wiringPi.h>
#include <unistd.h>

using namespace std;

class CAN0_Alternate
{
public:
    CAN0_Alternate(int baud);
    void CAN0pushPropDC(int dcG, int dcD);
    // string uint8_to_hex_string(const uint8_t *v, const size_t s);
    void CAN0ctrl_motor(int state);
    void CAN0ctrl_led(int state);
    //string int_to_hex_string(int theInt);
    // void CAN0configure(int baud);
    // string hexStr2(unsigned char *data, int len);
    // string int_to_hex(int a);
    void CAN0close();
    void msgClear(can_frame *fr);

private:
    int s;
    struct sockaddr_can addr;
    struct can_filter rfilter;
    struct can_frame frame;
    struct ifreq ifr;
    int BR;
};