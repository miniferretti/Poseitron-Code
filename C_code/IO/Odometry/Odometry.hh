#include "IO/Speed_Controller/CtrlStruct.hh"
#include <pthread.h>
#include <chrono>
#include <wiringPiSPI.h>
#include <math.h>
#include <iostream>
#include "stdio.h"

#define STEP 50 

class Odometry
{
public:
    Odometry(CtrlStruct *theCtrlStruct);
    void Odometry_start();
    void Odometry_init();
    void update_rot(unsigned char buffer);

    //Thread launching function
    static void *Odometry_update(void *daOdometry);

private:
    CtrlStruct *theCtrlStruct;
    pthread_t tr;
    pthread_mutex_t mutex1;
};
