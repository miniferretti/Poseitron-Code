
#include "CtrlStruct.hh"
#include "IO/COM/CAN/CAN_Alternate.hh"
#include <pthread.h>

class SpeedController
{
public:
    SpeedController(CtrlStruct *theCtrlStruct, CAN0_Alternate *can0);
    void init_speed_controller(int i);
    void run_speed_controller();
    void *updateLowCtrl(void *unused);
    void updateSpeed();
    void updateCmd();
    void PIController(MotStruct *theMot, double V_ref, double V_wheel_mes, double t);
    int saturation(double upperLimit, double lowerLimit, double *u);
    void speed_controller_active(int i);

private:
    CtrlStruct *theCtrlStruct;
    CAN0_Alternate *can0;
    pthread_t tr;
};