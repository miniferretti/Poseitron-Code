#ifndef PINCHERS_CONTROL_HH
#define PICNHERS_CONTROL_HH

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "IO/Speed_Controller/CtrlStruct.hh"
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

void set_pinchers_output(CtrlStruct *cvs, uint8_t outputL,uint8_t outputR);

#endif