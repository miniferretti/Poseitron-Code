/*************************************************************************
* Header file for DynamixelFunctions                                          *
* Version 1.00                                                           *
**************************************************************************/
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
#include <MyDynamixel.h>
/*************************************************************************
* Types Definitions					                                     *
**************************************************************************/
typedef unsigned char Byte; //integer of 8 bits

/*************************************************************************
* Functions list				                                     *
**************************************************************************/
void DynLightLed(); 
void SetMaxSpeed();
void SetMinAngle();
void SetMaxAngle();
