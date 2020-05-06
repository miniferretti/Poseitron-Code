#include "Pincher_Demo.hh"

void pincher_demo(CtrlStruct *cvs)
{

    float r, g, b, c; // 0 = red-dominant 1 = green-dominant 2 = blue-dominant
    uint16_t color_temp;
    RobotPinchers *pinch = cvs->pinchers;

    switch (cvs->pinchers_demo_states)
    {
    case SETUP_STATE:
        Dyn_set_position_and_speed(0x08, 0, 10);
        pinch->pinch_flag = 0;

        if (Dyn_get_position(0x08) == 0)
        {
            if (Dyn_set_torque(0x08, 100)) //Set a maximum torque for grabing the component
            {
                cvs->pinchers_demo_states = PAUSE_STATE2;
                cvs->t_ref = cvs->theCtrlIn->t;
            }
        }

        break;

    case SENS_STATE:
        sensorSelect(1);
        getRGB(&r, &g, &b);
        if (getColor(r, g, b) == 1)
        {
            printf("The cup is GREEN\r\n");
            if (pinch->pinch_flag == 0)
            {
                pinch->number_of_succes++;
            }
        }
        else if (getColor(r, g, b) == 0)
        {
            printf("The cup is RED\r\n");
        }
        else
        {
            printf("The cup is BLUE\r\n");
        }

        if (pinch->pinch_flag == 0)
        {
            pinch->number_of_pinch++;
        }
        pinch->pinch_flag = 1;

        printf("Number of pinch: %d Number of succes: %d\r\n", pinch->number_of_pinch, pinch->number_of_succes);

        cvs->pinchers_demo_states = PAUSE_STATE;
        break;

    case GRAB_STATE:
        // printf("the color temperature is: %d \r\n", color_temp);

        if (Dyn_set_position_and_speed(0x08, 300, 10))
        {
            cvs->pinchers_demo_states = LOAD_STATE;
        }
        break;

    case PAUSE_STATE:

        if ((cvs->theCtrlIn->t - cvs->t_ref) > 30)
        {
            cvs->pinchers_demo_states = SETUP_STATE;
        }
        else
        {
            cvs->pinchers_demo_states = SENS_STATE;
        }

        break;

    case LOAD_STATE:

        printf("The load is : %d\r\n", Dyn_get_load(0x08));
        if (Dyn_get_load(0x08) > 1100)
        {
            cvs->pinchers_demo_states = SENS_STATE;
            cvs->t_ref = cvs->theCtrlIn->t;
        }
        break;

    case PAUSE_STATE2:
        if ((cvs->theCtrlIn->t - cvs->t_ref) > 10)
        {
            cvs->pinchers_demo_states = GRAB_STATE;
        }
        break;

    default:
        break;
    }
}

int getColor(float r, float g, float b)
{

    float result = std::max({r, g, b});
    if (result == r)
        return 0;
    if (result == g)
        return 1;
    else
        return 2;
}