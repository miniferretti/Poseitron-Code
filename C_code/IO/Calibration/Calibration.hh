#ifndef CALIBRATION_HH
#define CALIBRATION_HH

#include "IO/Speed_Controller/CtrlStruct.hh"
#include "IO/Sens_Array/Sens_Array.hh"
#include "IO/Speed_Controller/ctrl_io.h"
#include "IO/Speed_Controller/SpeedController.hh"
#include "IO/Odometry/Odometry.hh"
#include "IO/Mid_level_controller/Avoid150.hh"
#include "math.h"

enum
{
    CALIB_1,
    CALIB_2,
    CALIB_3,
    CALIB_4
};

void calibration(CtrlStruct *cvs, SpeedController *spc, Odometry *odo);

#endif