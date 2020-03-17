#include "IO/Speed_Controller/CtrlStruct.hh"
#include "IO/COM/CAN/CAN_Alternate.hh"
#include <pthread.h>

class Behavior
{
    public:
    Behavior(CtrlStruct *theCtrlStruct, CAN0_Alternate *can0);
    void ObsUpdate();
    void runBehavior();
    void soul();
    void startBehavior();
    
    CtrlStruct *theCtrlStruct;
    CAN0_Alternate *can0;
    pthread_t tr;
}