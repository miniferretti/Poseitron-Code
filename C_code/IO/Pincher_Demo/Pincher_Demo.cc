#include "Pincher_Demo.hh"

void pincher_demo(CtrlStruct *cvs)
{

    uint16_t r, g, b, c; // 0 = red-dominant 1 = green-dominant 2 = blue-dominant
    uint16_t color_temp;

    switch (cvs->pinchers_demo_states)
    {
    case SETUP_STATE:

        sensorSelect(1);
        Dyn_set_position_and_speed(0x08, 0, 10);

        if (Dyn_get_position(0x08) == 0)
        {
            cvs->pinchers_demo_states = SENS_STATE;
        }

        break;

    case SENS_STATE:
        getRawData(&r, &g, &b, &c);
        color_temp = calculateColorTemperature(r, g, b);
        printf("the color temperature is: %d", color_temp);
        break;

    default:
        break;
    }
}
