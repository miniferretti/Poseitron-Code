///////////////////////////////////////////////////////////////////////
//
// Written by: Matteo Ferretti di Castelferretto
// 
//
///////////////////////////////////////////////////////////////////////

//
// Function similar to sensorSelect from the color sensor library but 
// activate or deactivate the MCP23017 outputs 
//


#include "Pinchers_control.hh"

void set_pinchers_output(unsigned char A, unsigned char B)
{

    unsigned char buffer[5];
    buffer[0] = PINCHER_REG;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    buffer[3] = B;
    buffer[4] = A;

    wiringPiSPIDataRW(0, buffer, 5);
}