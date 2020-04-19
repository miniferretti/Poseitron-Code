
#include "DynamixelFunctions.h"

/*************************************************************************
* This small librairy contains functions to control the dynamixel. 
* First control the led to test communication. 
  
  Next functions have been developped following the needs of the PoseiTron
  project (ndlr controlling the dynamixel along a rack and pinion). 				                                     *
**************************************************************************/

int DynLightLed(Byte ID)
{

  Byte Instruction = 0x03; // writes the data in the control table Write_reg
  Byte Length = 4;         //
  Byte P1 = 0x19;          // Starting address of the location where data is to be written here : LED address
  Byte P2 = 0x01;          // 1st data to be written : LED is lighten up
  Byte P3 = 0x00;
  Byte Error;
  Byte Checksum;
  Byte Fail;
  Byte ID1; // 2sd data to be written : not needed here

  int error;

  printf("ID is = %u\r\n", ID);
  // reset_dynamixel();

  Fail = Set_Parameter(ID, 4, 0x19, 0x01);
  // Send_Instruction_Packet(ID, 2, 0x05, 0, 0, 0);

  if (!Fail)
  {
    printf("Allumage de la LED \r\n");

    return 1;
  }
  else
  {
    printf("Erreur lors de l'envois du message\r\n");
    return 0;
  }
  //  printf("Dynamixel available = %d\r\n", read_data(COMMUNICATION_STATUS_REG));
}

int DynOffLED(Byte ID)
{
  Byte Fail;
  Fail = Set_Parameter(ID, 4, 0x19, 0x0);
  if (!Fail)
  {
    printf("Eteignage de la LED \r\n");

    return 1;
  }
  else
  {
    printf("Erreur lors de l'envois du message\r\n");
    return 0;
  }
}

void DynSetMaxSpeed(Byte ID, int Speed)
{
}

void DynSetMinAngle(Byte ID, int Angle)
{
}

void DynSetMaxAngle(Byte ID, int Angle)
{
}