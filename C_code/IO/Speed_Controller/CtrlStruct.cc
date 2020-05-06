#include "CtrlStruct.hh"
#include "IO/Mid_level_controller/Avoid150.hh"
#include "IO/Pincher_Demo/Pincher_Demo.hh"
#include "IO/Calibration/Calibration.hh"
#include "IO/Calibration/odo_calibration.h"

int size_UserStruct()
{
    return sizeof(CtrlStruct) * 10;
}

void init_ctrlStruc(CtrlStruct *ctrl)
{
    ctrl->theCtrlIn = new CtrlIn;
    ctrl->theUserStruct = new UserStruct;
    ctrl->theCtrlOut = new CtrlOut;
    ctrl->theUserStruct->theMotLeft = new MotStruct;
    ctrl->theUserStruct->theMotRight = new MotStruct;
    ctrl->rob_pos = new RobotPosition;
    ctrl->robot = new RobotParameters;
    ctrl->pinchers = new RobotPinchers;
    ctrl->pinchers->number_of_succes = 0;
    ctrl->pinchers->number_of_pinch = 0;
    ctrl->pinchers->pinch_flag = 0;
    ctrl->avoid150_states = AVOID150_STATE1;
    ctrl->pinchers_demo_states = SETUP_STATE;
    ctrl->calib_states = CALIB_1;
    ctrl->odo_calibration_states = GO_STRAIGHT;
    ctrl->main_t_ref = 0;
    ctrl->pinchers->RGBLog = fopen("/home/pi/Poseitron-Code/Data/RGBLog.txt", "w");
}