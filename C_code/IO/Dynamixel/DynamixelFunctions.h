/*************************************************************************
* Header file for DynamixelFunctions                                          *
* Version 1.00                                                           *
**************************************************************************/
#ifndef DYNAMIXELFUNCTIONS_H
#define DYNAMIXELFUNCTIONS_H
/*************************************************************************
* Library declarations				                                     *
**************************************************************************/
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "MyDynamixel.h"
/*************************************************************************
* Types Definitions					                                     *
**************************************************************************/
typedef unsigned char Byte; //integer of 8 bits

/*************************************************************************
* Functions list				                                     *
**************************************************************************/
int Dyn_light_LED(Byte ID);
void SetMaxSpeed();
void SetMinAngle();
void SetMaxAngle();
int Dyn_off_LED(Byte ID);
int Dyn_set_position_and_speed(Byte ID, int position, int speed);
int Dyn_get_position(Byte ID);
#endif