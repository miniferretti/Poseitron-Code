#ifndef ODO_CALIBRATION_H
#define ODO_CALIBRATION_H

#include "IO/Speed_Controller/CtrlStruct.hh"
#include "IO/Odometry/Odometry.hh"
#include "IO/Speed_Controller/SpeedController.hh"
#include "IO/Speed_Controller/ctrl_io.h"


enum{
        GO_STRAIGHT,
        GO_CIRCLE,
        GO_STOP
};

void odo_calibration(CtrlStruct *cvs,SpeedController *spc,Odometry *myOdo);

#endif