#ifndef BEHAVIOR_HH
#define BEHAVIOR_HH

#include "IO/Speed_Controller/CtrlStruct.hh"
#include "IO/Sens_Array/Sens_Array.hh"
#include "IO/Speed_Controller/ctrl_io.h"
#include "IO/Speed_Controller/SpeedController.hh"
#include "IO/Odometry/Odometry.hh"
#include "math.h"

    void avoid150(CtrlStruct *cvs, SpeedController *spc, Odometry *odo);
#endif