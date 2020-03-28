
#ifndef SPEEDCONTROLLER_HH
#define SPEEDCONTROLLER_HH

#include "CtrlStruct.hh"
#include "IO/COM/CAN/CAN_Alternate.hh"
#include <pthread.h>
#include <unistd.h>

#define MVG_LENG 1

class SpeedController
{
public:
    SpeedController(CtrlStruct *theCtrlStruct, CAN0_Alternate *can0,pthread_mutex_t *theMutex); //Constructor
    void init_speed_controller(int i);                                // injecte toutes les valeurs utiles dans la strcuture de controle
    void run_speed_controller();                                      // fonction a lancer pour demarrer les speed controller

    //Thread launching fucntion
    static void *updateLowCtrl(void *daSpeedController);                                //looping function inside the speedcontroller thread
    void updateSpeed(unsigned char *buffer);                                            //update of the speed
    void updateCmd();                                                                   // update of the command
    double PIController(MotStruct *theMot, double V_ref, double V_wheel_mes, double t); //PI speed regulator
    int saturation(double upperLimit, double lowerLimit, double *u);
    void speed_controller_active(int i);
    double Moving_Average(double speed, double *buff, int leng);

    CtrlStruct *theCtrlStruct;
    CAN0_Alternate *can0;
    pthread_t tr;

    double avgR[MVG_LENG];
    double avgL[MVG_LENG];


    pthread_mutex_t *theMutex;
};

#endif