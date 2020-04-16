#include "CtrlStruct.hh"

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

}