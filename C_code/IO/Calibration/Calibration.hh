#ifndef CALIBRATION_HH
#define CALIBRATION_HH

#include "IO/Speed_Controller/CtrlStruct.hh"
#include "IO/Sens_Array/Sens_Array.hh"
#include "IO/Speed_Controller/ctrl_io.h"
#include "IO/Speed_Controller/SpeedController.hh"
#include "math.h"

void calibration(CtrlStruct *cvs, SpeedController *spc);

#endif