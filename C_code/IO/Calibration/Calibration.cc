#include "IO/Calibration/Calibration.hh"

void calibration(CtrlStruct *cvs, SpeedController *spc)
{
    CtrlIn *myCtrlIn = cvs->theCtrlIn;

    switch (cvs->calib_states)
    {
    case CALIB_1:
        printf("CALIB_1\r\n");
        spc->set_speed(10, 10);
        get_prox(spc->can0, cvs);

        if (((0 < myCtrlIn->sens_array_front[1]) && (myCtrlIn->sens_array_front[1] < 80)) || ((0 < myCtrlIn->sens_array_front[3]) && (myCtrlIn->sens_array_front[3] < 80)))
        {
            spc->set_speed(0, 0);
            cvs->calib_states = CALIB_2;
        }

        break;

    case CALIB_2:
        printf("CALIB_2\r\n");
        get_prox(spc->can0, cvs);
        if (abs((myCtrlIn->sens_array_front[1] - myCtrlIn->sens_array_front[3])) < 10)
        {
            spc->set_speed(0, 0);
            cvs->calib_states = CALIB_3;
        }
        else if ((myCtrlIn->sens_array_front[1] - myCtrlIn->sens_array_front[3]) > 0)
        {
            spc->set_speed(5, 0);
        }
        else
        {
            spc->set_speed(0, 5);
        }

        break;

    case CALIB_3:
        printf("CALIB_3\n\r");
        cvs->calib_states = CALIB_1;
        cvs->main_states = STOP_STATE;
        break;

    default:
        break;
    }
}