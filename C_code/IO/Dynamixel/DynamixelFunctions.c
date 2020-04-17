#include "MyDynamixel.h"
#include "DynamixelFunctions.h"

/*************************************************************************
* This small librairy contains functions to control the dynamixel. 
* First control the led to test communication. 
  
  Next functions have been developped following the needs of the PoseiTron
  project (ndlr controlling the dynamixel along a rack and pinion). 				                                     *
**************************************************************************/

void DynLed(Byte ID){

    Byte Instruction = 0x03;    // writes the data in the control table
    Byte Length = ;             //
    Byte P1 = 0x19;             // Starting address of the location where data is to be written here : LED address
    Byte P2 = 0x01;             // 1st data to be written : LED is lighten up
    Byte P3 = 0x00;             // 2sd data to be written : not needed here
    
    int error; 

    error = Send_Instruction_Packet(ID, Length, Instruction, P1, P2, P3);

    if(error==1){
        printf("Si la LED est pas allumée, ya une couille dans le potage...");
    }
    else{
        printf("Comment dire chef, ya eu comme une erreur quand j'ai envoyé l'instruction chef!");
    }
}

void DynSetMaxSpeed(Byte ID, int Speed){

}

void DynSetMinAngle(Byte ID, int Angle){

}

void DynSetMaxAngle(Byte ID, int Angle){

}