#include <pthread.h>
#include <chrono>
#include <wiringPiSPI.h>
#include <math.h>
#include <iostream>
#include "stdio.h"
#include "Behavior.hh"

Behavior::Behavior(CtrlStruct *theCtrlStruct, CAN0_Alternate *can0){
    this->can0=can0;
    this->theCtrlStruct=theCtrlStruct;
}