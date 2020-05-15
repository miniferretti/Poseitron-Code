///////////////////////////////////////////////////////////////////////
//
// Written by: Eleonore Masarweh
// 
//
///////////////////////////////////////////////////////////////////////



/************************************************************************/
// Vision and mapping of the obstacles
// 
// This code takes the ultrasonic sensor data in input. 
// The output is a 360-vector covering a 360-vision, the same way a lidar
// would. 
// The code supposes that the average obstacle is 20 cm wide. 
/************************************************************************/
#include <math.h>
#include "CtrlStruct.hh"
#include "ctrl_io.h"

void vue_and_map(CtrlStruct* theCtrlStruct); 
void sensorwise_crossed_correction(int signal_nmbr, CtrlStruct *theCtrlStruct); 
void sensorwise_direct(int signal_nmbr, CtrlStruct *theCtrlStruct); 
void locate_obstacles (CtrlStruct * theCtrlStruct); 
void map_obstacles (CtrlStruct * theCtrlStruct);