
#ifndef SPEEDCONTROLLER_HH
#define SPEEDCONTROLLER_HH

#include "CtrlStruct.hh"
#include "IO/COM/CAN/CAN_Alternate.hh"
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
#include <netdb.h>

#define MVG_LENG 1
#define UDP_PORT 5005 // Port number for the UDP server

class SpeedController
{
public:
    SpeedController(CtrlStruct *theCtrlStruct, CAN0_Alternate *can0); //Constructor
    void init_speed_controller(int i);                                // injecte toutes les valeurs utiles dans la strcuture de controle
                                                                      // fonction a lancer pour demarrer les speed controller

    //Thread launching fucntion
    void updateLowCtrl();                                                               //looping function inside the speedcontroller thread
    void updateSpeed(unsigned char *buffer);                                            //update of the speed
    void updateCmd();                                                                   // update of the command
    double PIController(MotStruct *theMot, double V_ref, double V_wheel_mes, double t); //PI speed regulator
    int saturation(double upperLimit, double lowerLimit, double *u);
    void speed_controller_active(int i);
    double Moving_Average(double speed, double *buff, int leng);
    void set_speed(double left, double right);
    void Speed_controller_stop();

private:
    CtrlStruct *theCtrlStruct;
    CAN0_Alternate *can0;

    double avgR[MVG_LENG];
    double avgL[MVG_LENG];
    int counter;

    FILE *logFile;
    FILE *PIDFile;

    int sock, length, n, slave, slave_previous_state, ID_type;
    socklen_t fromlen;
    char UDP_speed_msg[40];
    struct sockaddr_in servaddr;
    struct sockaddr_in from;
    struct timeval read_timeout;
    double t_ref;
    float kp_left, ki_left, kd_left, kp_right, ki_right, kd_right, correction_factor_left, correction_factor_right, slave_speed_left, slave_speed_right, omega_sat, speed_sat, prop_param, rho_limit;
};

#endif