///////////////////////////////////////////////////////////////////////
//
// Written by: Matteo Ferretti di Castelferretto
// 
//
///////////////////////////////////////////////////////////////////////


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
#define PINCHER_REG 0x88

void set_pinchers_output(unsigned char A, unsigned char B);

#endif