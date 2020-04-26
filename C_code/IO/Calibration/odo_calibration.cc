#include "odo_calibration.h"

void odo_calibration(CtrlStruct *cvs, SpeedController *spc, Odometry *myOdo)
{
    CtrlIn *inputs = cvs->theCtrlIn;
    double limit = 30;

    switch (cvs->odo_calibration_states)
    {
    case GO_STRAIGHT:

        spc->set_speed(30, 30);
        printf("left dist = %f right dist = %f\r\n", myCtrlIn->sens_array_front[1], myCtrlIn->sens_array_front[3]);
        if ((((0 < myCtrlIn->sens_array_front[1]) && (myCtrlIn->sens_array_front[1] < limit)) || ((0 < myCtrlIn->sens_array_front[3]) && (myCtrlIn->sens_array_front[3] < limit))) && cvs->theCtrlIn->sens_flag == 1)
        {
            cvs->odo_calibration_states = GO_STOP;
            printf("Obstacle ptn !\r\n");
        }

        break;

    case GO_STOP:
            spc->set_speed(0,0);
            cvs->main_states = STOP_STATE;
        break;

    default:
        break;
    }
}