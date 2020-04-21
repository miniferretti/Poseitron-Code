#include "Pincher_Demo.hh"

void pincher_demo(CtrlStruct *cvs)
{

    uint16_t r, g, b, c; // 0 = red-dominant 1 = green-dominant 2 = blue-dominant
    uint16_t color_temp;

    switch (cvs->pinchers_demo_states)
    {
    case SETUP_STATE:
        Dyn_set_position_and_speed(0x08, 0, 10);

        if (Dyn_get_position(0x08) == 0)
        {
            cvs->pinchers_demo_states = SENS_STATE;
        }

        break;

    case SENS_STATE:
        sensorSelect(1);
        getRawData(&r, &g, &b, &c);
        color_temp = calculateColorTemperature(r, g, b);
        printf("the color temperature is: %d \r\n", color_temp);

        if (5000 < color_temp && color_temp < 8000) //Color detected is green
        {
            if (Dyn_set_torque(0x08, 100)) //Set a maximum torque for grabing the component
            {
                cvs->pinchers_demo_states = GRAB_STATE;
            }
        }
        break;

    case GRAB_STATE:
        printf("the color temperature is: %d \r\n", color_temp);

        if (Dyn_set_position_and_speed(0x08, 200, 10))
        {
            cvs->pinchers_demo_states = PAUSE_STATE;
            cvs->t_ref = cvs->theCtrlIn->t;
        }
        break;

    case PAUSE_STATE:
        printf("The load is : %d", Dyn_get_load(0x08));
        if ((cvs->theCtrlIn->t - cvs->t_ref) > 20)
        {
            cvs->pinchers_demo_states = SETUP_STATE;
        }

        break;

    default:
        break;
    }
}
