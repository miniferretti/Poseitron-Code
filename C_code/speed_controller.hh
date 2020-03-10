#include "CtrlStruct.h"
#include "IO/ctrl_io.h"

void run_speed_controller(CtrlStruct *theCtrlStruct);
void init_speed_controller(CtrlStruct *theCtrlStruct);
int saturation(double upperLimit, double lowerLimit, double *u);