#include "CtrlStruct.hh"
#include "ctrl_io.h"

int size_UserStruct()
{
    return sizeof(CtrlStruct) * 10;
}

void init_ctrlStruc(CtrlStruct *ctrl)
{

    ctrl->theCtrlIn = new CtrlIn;
    ctrl->theUserStruct = new UserStruct;
    ctrl->theCtrlOut = new CtrlOut;
}