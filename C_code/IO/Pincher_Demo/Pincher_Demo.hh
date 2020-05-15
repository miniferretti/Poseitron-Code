///////////////////////////////////////////////////////////////////////
//
// Written by: Matteo Ferretti di Castelferretto
// 
//
///////////////////////////////////////////////////////////////////////

// This behavior is only used to validate the working principle of the 
// 3D printed Dynamixel based plier. It uses the color sensors and the 
// Dynamixel libraries. 


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
#include <initializer_list>
#include <algorithm>

enum
{
    SETUP_STATE,
    SENS_STATE,
    PAUSE_STATE,
    PAUSE_STATE2,
    GRAB_STATE,
    LOAD_STATE,
    RELEASE_STATE
};

void pincher_demo(CtrlStruct *cvs);
int getColor(float r,float g, float b);

#endif