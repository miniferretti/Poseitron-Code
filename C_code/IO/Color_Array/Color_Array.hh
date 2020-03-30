#ifndef COLOR_ARRAY_HH
#define COLOR_ARRAY_HH

#include "IO/Speed_Controller/CtrlStruct.hh"
#include "IO/COM/TCS3472_I2C/TCS3472_I2C.hh"
#define NUMSENS 6
#define NUMPERSIDE 3

void update_color_array(Adafruit_TCS34725 *myColorSensor, CtrlStruct *cvs);

#endif