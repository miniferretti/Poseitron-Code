#include "Avoid150.hh"

void avoid150(CtrlStruct *cvs, SpeedController *spc, Odometry *odo)
{
    CtrlIn *myCtrlIn = cvs->theCtrlIn;
    RobotPosition *pos = cvs->rob_pos;

    switch (cvs->avoid150_states)
    {
    case AVOID150_STATE1:

        if (myCtrlIn->sens_array_front[1] < 40 && myCtrlIn->sens_array_front[3] < 40)
        {
            pos->thetaref = pos->theta; //stock l'angle actuel
            cvs->avoid150_states = AVOID150_STATE2;
        }
        else
        {
            cvs->main_states = CALIB_STATE;
        }

        break;

    case AVOID150_STATE2:

        spc->set_speed(-10, 10);

        if ((pos->theta - pos->thetaref) > 6.28)
        {
            cvs->avoid150_states = AVOID150_STATE3;
        }

        break;

    case AVOID150_STATE3:
        cvs->main_states = CALIB_STATE;
        cvs->avoid150_states = AVOID150_STATE1;
        break;

    default:
        break;
    }
}