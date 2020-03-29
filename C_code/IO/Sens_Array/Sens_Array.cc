#include "IO/COM/CAN/CAN_Alternate.hh"
#include "IO/Speed_Controller/CtrlStruct.hh"
#include "Sens_Array.hh"

void get_prox(can *can0, CtrlStruct *cvs)
{
    can0->getDistance(1, cvs->theCtrlIn->sens_array_front);
    can0->getDistance(0, cvs->theCtrlIn->sens_array_back);
}