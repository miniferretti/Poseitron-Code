#ifndef SENS_ARRAY_HH
#define SENS_ARRAY_HH

#include "IO/COM/CAN/CAN_Alternate.hh"
#include "IO/Speed_Controller/CtrlStruct.hh"


int get_prox(CAN0_Alternate *can0, CtrlStruct *cvs);

#endif