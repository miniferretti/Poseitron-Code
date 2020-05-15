///////////////////////////////////////////////////////////////////////
//
// Written by: Matteo Ferretti di Castelferretto
// 
//
///////////////////////////////////////////////////////////////////////

#include "IO/Calibration/Calibration.hh"

void calibration(CtrlStruct *ctrl, SpeedController *spc, Odometry *odo)
{
    CtrlIn *myCtrlIn = ctrl->theCtrlIn;
    double limit = 30;

    switch (ctrl->calib_states)
    {
    case CALIB_1:
        printf("CALIB_1\r\n");
        spc->set_speed(30, 30);
        //  flag = get_prox(spc->can0, ctrl);

        printf("left dist = %f right dist = %f\r\n", myCtrlIn->sens_array_front[1], myCtrlIn->sens_array_front[3]);
        if ((((0 < myCtrlIn->sens_array_front[1]) && (myCtrlIn->sens_array_front[1] < limit)) || ((0 < myCtrlIn->sens_array_front[3]) && (myCtrlIn->sens_array_front[3] < limit))) && ctrl->theCtrlIn->sens_flag == 1)
        {
            ctrl->calib_states = CALIB_2;
            printf("Obstacle ptn !\r\n");
        }

        break;

    case CALIB_2:
        printf("CALIB_2\r\n");
        printf("left dist = %f right dist = %f\r\n", myCtrlIn->sens_array_front[1], myCtrlIn->sens_array_front[3]);
        //  flag = get_prox(spc->can0, ctrl);
        if ((abs((myCtrlIn->sens_array_front[1] - myCtrlIn->sens_array_front[3])) < 2))
        {

            if ((myCtrlIn->sens_array_front[1] > limit) && (myCtrlIn->sens_array_front[3] > limit))
            {
                ctrl->calib_states = CALIB_1;
            }
            else
            {
                spc->set_speed(0, 0);
                ctrl->stopvalues[0] = myCtrlIn->sens_array_front[1];
                ctrl->stopvalues[1] = myCtrlIn->sens_array_front[3];
                ctrl->calib_states = CALIB_3;
            }
        }
        else if (((myCtrlIn->sens_array_front[1] - myCtrlIn->sens_array_front[3]) > 0))
        {
            spc->set_speed(0, 10);
            if (myCtrlIn->sens_array_front[1] > limit && myCtrlIn->sens_array_front[3] > limit)
            {
                ctrl->calib_states = CALIB_1;
            }
        }
        else
        {
            spc->set_speed(10, 0);
            if (myCtrlIn->sens_array_front[1] > limit && myCtrlIn->sens_array_front[3] > limit)
            {
                ctrl->calib_states = CALIB_1;
            }
        }

        break;

    case CALIB_3:
        printf("CALIB_3\n\r");
        ctrl->calib_states = CALIB_1;            // Reset previous behavior
        ctrl->main_states = AVOID150_STATE;      // Set next behavior
        ctrl->avoid150_states = AVOID150_STATE1; // Set the first state of the next behavior
        break;

    default:
        break;
    }
}

/*int go_straight(CtrlStruct *ctrl, double speed, double dist)
{
	Odometry *odo_pos;
	odo_pos = ctrl->odo_pos;

	if (odo_pos->ddist_flag == 0)
	{
		set_ddist(ctrl);
	}

	if (dist != 0)
	{
		if (get_ddist(ctrl) >= dist)
		{
			reset_ddist(ctrl);
			odo_pos->ignore = 2;
			//set_speed(ctrl, 0, 0);
			return 1;
		}
	}
	else
	{
		reset_ddist(ctrl);
	}

	double e = (odo_pos->dydx - odo_pos->dydx_prev) * 0.01; // simple proportionnal controller

	if (e > 1)
	{
		e = 1;
	}
	if (e < -1)
	{
		e = -1;
	}

	double vr = (1 - e) * speed;
	double vl = 2 * speed - vr;

	if (odo_pos->ignore >= 1)
	{
		set_speed(ctrl, speed, speed);
		odo_pos->ignore--;
	}
	else
	{
		set_speed(ctrl, vl, vr);
	}
	return 0;
}

*/