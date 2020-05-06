
#include "IO/Sens_Array/Sens_Array.hh"

int get_prox(CAN0_Alternate *can0, CtrlStruct *cvs)
{
   return can0->getDistance(1, cvs->theCtrlIn->sens_array_front);
   // can0->getDistance(0, cvs->theCtrlIn->sens_array_back);
}