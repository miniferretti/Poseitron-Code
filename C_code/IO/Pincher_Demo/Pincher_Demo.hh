#ifndef PINCHER_DEMO_HH
#define PINCHER_DEMO_HH

#include "IO/Speed_Controller/CtrlStruct.hh"
#include "IO/Sens_Array/Sens_Array.hh"
#include "IO/Speed_Controller/ctrl_io.h"
#include "IO/Speed_Controller/SpeedController.hh"
#include "IO/Odometry/Odometry.hh"
#include "math.h"
#include "IO/COM/TCS3472_I2C/TCS3472_FPGA.hh"
#include "IO/Dynamixel/DynamixelFunctions.h"
#include <iostream>
#include <sstream>

enum
{
    SETUP_STATE,
    SENS_STATE,
    WAIT_STATE,
    GRAB_STATE,
    RELEASE_STATE
};

void pincher_demo(CtrlStruct *cvs);

#endif