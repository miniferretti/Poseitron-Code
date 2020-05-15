
///////////////////////////////////////////////////////////////////////
//
// Written by: Matteo Ferretti di Castelferretto
// 
//
///////////////////////////////////////////////////////////////////////

#include "IO/Color_Array/Color_Array.hh"

void update_color_array(CtrlStruct *cvs)
{

    float r;
    float g;
    float b;

    for (int i = 0; i < NUMSENS; i++)
    {
        if (i > (NUMPERSIDE - 1))
        {
            sensorSelect(i);
            getRGB(&r, &g, &b);
            cvs->theCtrlIn->color_array_right[i - NUMPERSIDE] = calculateColorTemperature((uint16_t)r, (uint16_t)g, (uint16_t)b);
        }
        else
        {
            sensorSelect(i);
            getRGB(&r, &g, &b);
            cvs->theCtrlIn->color_array_left[i] = calculateColorTemperature((uint16_t)r, (uint16_t)g, (uint16_t)b);
        }
    }
}