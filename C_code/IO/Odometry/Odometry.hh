#ifndef ODOMETRY_HH
#define ODOMETRY_HH

#include "IO/Speed_Controller/CtrlStruct.hh"
#include <chrono>
#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <math.h>
#include <iostream>
#include "stdio.h"
#include <unistd.h>

#define STEP 50

class Odometry
{
public:
    Odometry(CtrlStruct *theCtrlStruct);
    void Odometry_start();
    void Odometry_init();
    void update_rot(unsigned char *buffer);
    void Odometry_stop();
    void set_theta_ref();
    void reset_theta_ref();
    double get_dtheta();
    double get_ddist();
    void set_pos();
    void reset_pos();
    void set_ddist();
    void reset_ddist();
    void reset_odometry();

    //Thread launching function
    void Odometry_update();

private:
    FILE *logFile;
    CtrlStruct *theCtrlStruct;
};

#endif