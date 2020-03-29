#ifndef _SENS_ARRAY_HH
#define _SENS_ARRAY_HH

#include "IO/COM/CAN/CAN_Alternate.hh"
#include "IO/Speed_Controller/CtrlStruct.hh"

void get_prox(can *can0, CtrlStruct *cvs);

#endif