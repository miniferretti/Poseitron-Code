
#include "DynamixelFunctions.h"

/*************************************************************************
* This small librairy contains functions to control the dynamixel. 
* First control the led to test communication. 
  
  Next functions have been developped following the needs of the PoseiTron
  project (ndlr controlling the dynamixel along a rack and pinion). 				                                     *
**************************************************************************/

int Dyn_light_LED(Byte ID)
{
  Byte Fail;

  printf("ID is = %u\r\n", ID);  //Print l'ID juste pour vérfier

  Fail = Set_Parameter(ID, 4, LED_REG, 0x01); //Fonction d'evois de commande qui prend un regsitre et une consigne en arguments. Si la consigne est plus grand que un byte, passer le 4 à 5.

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
}

int Dyn_off_LED(Byte ID) 
{
  Byte Fail;
  Fail = Set_Parameter(ID, 4, LED_REG, 0x0);
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

int Dyn_set_position_and_speed(Byte ID, int postion, int speed) //Speed in rpm and position in a range from 0 to 1023 --> from 15 deg to 345 deg
{
  Byte Fail;

  speed = speed * 10;

  Set_Parameter(ID, 5, MOVING_SPEED_REG, speed); //S'occupe de definir la vitesse du dynamixel. La vitesse s'etalle sur 2 bytes, donc 4 devient 5. 

  Fail = Set_Parameter(ID, 5, GOAL_POSITION_REG, postion); //Leght = 5 car la postion est étalée sur P2 et P3 donc 3+2 avec P1 qui contient l'adresse du registre.

  if (!Fail) //Verification condition
  {
    printf("Position enregistrée \r\n");

    return 1;
  }
  else
  {
    printf("Erreur lors de l'envois du message\r\n");
    return 0;
  }
}

int Dyn_get_position(Byte ID) // Fonction qui s'occupe de lire le registre de postion du dynamixel
{
  int Fail;

  Fail = Get_Parameters(ID, PRESENT_POSITION_REG, 2); // Fonction utilisée pour lire un registre du dynamixel ici la postion max est 0x1FE donc peut s'étaller sur deux bytes d'ou le 2//

  return Fail;
}

int Dyn_torque_en(Byte ID, int en) //Fonction qui active la limite en torque du dynamixel. en = 1 ou 0
{
  Byte Fail;
  Fail = Set_Parameter(ID, 4, TORQUE_ENABLE, en);
  if (!Fail)
  {
    printf("Torque activé \r\n");

    return 1;
  }
  else
  {
    printf("Erreur lors de l'envois du message\r\n");
    return 0;
  }
}

int Dyn_set_torque(Byte ID, int MaxTorque) //Fonction qui specifie le torque maximum du dynamixel
{
  Byte Fail;
  Fail = Set_Parameter(ID, 5, TORQUE_LIMIT_REG, MaxTorque);
  if (!Fail)
  {
    printf("Torque maximum enregistré \r\n");

    return 1;
  }
  else
  {
    printf("Erreur lors de l'envois du message\r\n");
    return 0;
  }
}

int Dyn_get_load(Byte ID) //Fonction qui spécifie le torque actuellement appliqué sur le dynamixel
{
  int Fail;
  Fail = Get_Parameters(ID, PRESENT_LOAD_REG, 2); // La valeur de ce torque peut s'étaller sur 2 bytes, d'ou le 2
  return Fail;
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