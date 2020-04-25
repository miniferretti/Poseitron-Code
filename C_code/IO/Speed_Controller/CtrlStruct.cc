#include "CtrlStruct.hh"
#include "IO/Mid_level_controller/Avoid150.hh"
#include "IO/Pincher_Demo/Pincher_Demo.hh"
#include "IO/Calibration/Calibration.hh"

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
    ctrl->avoid150_states = AVOID150_STATE1;
    ctrl->pinchers_demo_states = SETUP_STATE;
    ctrl->calib_states = CALIB_1;
}