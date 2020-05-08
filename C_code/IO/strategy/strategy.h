

#ifndef _STRATEGY_H_
#define _STRATEGY_H_

#include "IO/Speed_Controller/CtrlStruct.hh"
#include <math.h>
#include "IO/Mid_level_controller/path_follow.h"
#include "IO/Speed_Controller/SpeedController.hh"

//#define _GNU_SOURCE

// function prototype
void main_strategy(CtrlStruct *ctrl, P_Struct *my_P_Struct);

int opponent_detection(CtrlStruct *ctrl);

#endif
