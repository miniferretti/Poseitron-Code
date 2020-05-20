
///////////////////////////////////////////////////////////////////////
//
// Written by: Matteo Ferretti di Castelferretto
// 
//
///////////////////////////////////////////////////////////////////////

#pragma once
//#ifndef CAN_ALTERANTE_HH
//#define CAN_ALTERNATE_HH

#define CAN_MOT 0x508
#define CAN_SENS_ARRAY_FRONT 0x600
#define CAN_SENS_ARRAY_BACK 0x601
#define CAN_ID 0x100
#define DELAY 50
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
#include <fcntl.h>

using namespace std;

class CAN0_Alternate
{
public:
    CAN0_Alternate(int baud);
    void CAN0pushPropDC(double dcG, double dcD);
    void CAN0ctrl_motor(int state);
    void CAN0ctrl_led(int state);
    void CAN0close();
    void msgClear(can_frame *fr);
    int getDistance(int dir, double *data);
    int fd_set_blocking(int fd, int blocking);

private:
    int s;
    struct sockaddr_can addr;
    struct can_filter rfilter;
    struct can_frame frame;
    struct ifreq ifr;
    struct timeval tv;
    int BR;
};

//#endif