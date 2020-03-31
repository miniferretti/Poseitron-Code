#include "IO/Calibration/Calibration.hh"

void calibration(CtrlStruct *cvs, SpeedController *spc, Odometry *odo)
{
    CtrlIn *myCtrlIn = cvs->theCtrlIn;
    double limit = 30;
    

    switch (cvs->calib_states)
    {
    case CALIB_1:
        printf("CALIB_1\r\n");
        spc->set_speed(30, 30);
        //  flag = get_prox(spc->can0, cvs);

        printf("left dist = %f right dist = %f\r\n", myCtrlIn->sens_array_front[1], myCtrlIn->sens_array_front[3]);
        if ((((0 < myCtrlIn->sens_array_front[1]) && (myCtrlIn->sens_array_front[1] < limit)) || ((0 < myCtrlIn->sens_array_front[3]) && (myCtrlIn->sens_array_front[3] < limit))) && cvs->theCtrlIn->sens_flag == 1)
        {
            cvs->calib_states = CALIB_2;
            printf("Obstacle ptn !\r\n");
        }

        break;

    case CALIB_2:
        printf("CALIB_2\r\n");
        printf("left dist = %f right dist = %f\r\n", myCtrlIn->sens_array_front[1], myCtrlIn->sens_array_front[3]);
        //  flag = get_prox(spc->can0, cvs);
        if ((abs((myCtrlIn->sens_array_front[1] - myCtrlIn->sens_array_front[3])) < 2))
        {

            if ((myCtrlIn->sens_array_front[1] > limit) && (myCtrlIn->sens_array_front[3] > limit))
            {
                cvs->calib_states = CALIB_1;
            }
            else
            {
                spc->set_speed(0, 0);
                cvs->stopvalues[0] = myCtrlIn->sens_array_front[1];
                cvs->stopvalues[1] = myCtrlIn->sens_array_front[3];
                cvs->calib_states = CALIB_3;
            }
        }
        else if (((myCtrlIn->sens_array_front[1] - myCtrlIn->sens_array_front[3]) > 0))
        {
            spc->set_speed(0, 10);
            if (myCtrlIn->sens_array_front[1] > limit && myCtrlIn->sens_array_front[3] > limit)
            {
                cvs->calib_states = CALIB_1;
            }
        }
        else
        {
            spc->set_speed(10, 0);
            if (myCtrlIn->sens_array_front[1] > limit && myCtrlIn->sens_array_front[3] > limit)
            {
                cvs->calib_states = CALIB_1;
            }
        }

        break;

    case CALIB_3:
        printf("CALIB_3\n\r");
        cvs->calib_states = CALIB_1;
        cvs->main_states = AVOID150_STATE;
        cvs->avoid150_states = AVOID150_STATE1;
        break;

    default:
        break;
    }
}