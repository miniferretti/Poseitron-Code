#ifndef ODOMETRY_HH
#define ODOMETRY_HH

#include "IO/Speed_Controller/CtrlStruct.hh"
#include <pthread.h>
#include <chrono>
#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <math.h>
#include <iostream>
#include "stdio.h"
#include <unistd.h>

#define STEP 50

class Odometry
{
public:
    Odometry(CtrlStruct *theCtrlStruct,pthread_mutex_t *theMutex);
    void Odometry_start();
    void Odometry_init();
    void update_rot(unsigned char *buffer);
    void Odometry_stop();

    //Thread launching function
    static void *Odometry_update(void *daOdometry);

private:
    CtrlStruct *theCtrlStruct;
    pthread_t tr;
    pthread_mutex_t *theMutex;
};

#endif