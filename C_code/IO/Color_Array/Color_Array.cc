#include "IO/Color_Array/Color_Array.hh"

void update_color_array(Adafruit_TCS34725 *Sensor, CtrlStruct *cvs)
{

    float r;
    float g;
    float b;

    for (int i = 0; i < NUMSENS; i++)
    {
        if (i > (NUMPERSIDE - 1))
        {
            Sensor->sensorSelect(i);
            Sensor->getRGB(&r, &g, &b);
            cvs->theCtrlIn->color_array_right[i - NUMPERSIDE] = Sensor->calculateColorTemperature((uint16_t)r, (uint16_t)g, (uint16_t)b);
        }
        else
        {
            Sensor->sensorSelect(i);
            Sensor->getRGB(&r, &g, &b);
            cvs->theCtrlIn->color_array_left[i] = Sensor->calculateColorTemperature((uint16_t)r, (uint16_t)g, (uint16_t)b);
        }
    }
}