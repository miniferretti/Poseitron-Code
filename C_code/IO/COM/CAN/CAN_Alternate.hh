
#define CAN_MOT 0x508
#define CAN_SENS_ARRAY_FRONT 0x600
#define CAN_SENS_ARRAY_BACK 0x601
#define DELAY 5
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
    void CAN0ctrl_motor(int state);
    void CAN0ctrl_led(int state);
    void CAN0close();
    void msgClear(can_frame *fr);
    void getDistance(int dir, double *data);

private:
    int s;
    struct sockaddr_can addr;
    struct can_filter rfilter;
    struct can_frame frame;
    struct ifreq ifr;
    int BR;
};