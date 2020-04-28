#include <cstdio>
#include <stdio.h>

#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"

#include <iostream>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include "IO/Speed_Controller/SpeedController.hh"
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <string>
#include <sstream>
#include "IO/COM/TCS3472_I2C/TCS3472_I2C.hh"
#include <chrono>
#include <Dynamixel/MyDynamixel.h>

using namespace std;

//Constantes utiles
#define CAN_BR 500000
#define CS 0
#define RESETSPI 19

double omega_ref_now_r = 110;
double omega_ref_now_l = 100;

/*
	nom provisoire
	Inputs : r, radius of the turn in meter
			 v, speed at wich the robot needs to take the turn in m/s or rad/s
			 p, parameter to know what are the units of v. Must be 'm' or 'r'
	Outputs : void
	Bahaviour: The function calculates the speed of the two wheels and 
		store them in myCtrlStruct
*/
void simple_turn(double v, double r, char p) ;

void simple_turn(double v, double r, char p)  {
	double entreAxe = 189 ; // mm
	double wheelRadius = 30.15 ; // mm
	double mmToM = e-3 ; // coefficient to pass from mm to m
	double msToRads = 1/wheelRadius ;

	if(p == 'm'){ // cas de m/s
		myCtrlStruct->theCtrlIn->r_wheel_ref = msToRads * v * r /(mmToM * (ea/2 + r));
		myCtrlStruct->theCtrlIn->l_wheel_ref = msToRads * v * (ea + r)/(mmToM * (ea/2 + r));
	}
	else if(p == 'r'){ // cas de rad/s
		myCtrlStruct->theCtrlIn->r_wheel_ref = v * r /(mmToM * (ea/2 + r));
		myCtrlStruct->theCtrlIn->l_wheel_ref = v * (ea + r)/(mmToM * (ea/2 + r));
	}
	else{
		printf("Error : parameter p unvalid in fucntion simple_turn") ;
	}

}
	

int main()
{
	CtrlStruct *myCtrlStruct = new CtrlStruct;
	CAN0_Alternate *can = new CAN0_Alternate(CAN_BR);
	SPI_DE0 *deo;
	deo = new SPI_DE0(0, 125e3);

	delay(100);

	init_ctrlStruc(myCtrlStruct);

	SpeedController *spctrl = new SpeedController(myCtrlStruct, can);

	myCtrlStruct->theCtrlIn->r_wheel_ref = omega_ref_now_r;
	myCtrlStruct->theCtrlIn->l_wheel_ref = omega_ref_now_l;

	spctrl->init_speed_controller(1);

	spctrl->run_speed_controller();

	printf("Welcome to the Poseitron code prototype.\r\n");
	printf("We hope that you will be pleased with the coding and we wish you a great succes.\n\r");

	//********  Début du comportement du robot **********

	Byte dyn_error = 0x00;
	Byte ID = 0x06;
	Byte Length = 0x06; 
	Byte Address = 0x19;
	Byte Parameter = 0x01;

	while (true)
	{
		delay(10000);
		//	myCtrlStruct->theCtrlIn->r_wheel_ref = -10;
		//	myCtrlStruct->theCtrlIn->l_wheel_ref = -10;
		printf("wallah je suis la boucle du main !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
		dyn_error = Set_Parameter(ID, Length, Address, Parameter); 
		if(dyn_error!=)
		delay(10000);
		//myCtrlStruct->theCtrlIn->r_wheel_ref = omega_ref_now_r;
		//myCtrlStruct->theCtrlIn->l_wheel_ref = omega_ref_now_l;
	}
	
	free(myCtrlStruct->theUserStruct->theMotRight);
	free(myCtrlStruct->theUserStruct->theMotLeft);
	free(myCtrlStruct->theCtrlIn);
	free(myCtrlStruct->theCtrlOut);
	free(myCtrlStruct->theUserStruct);
	free(myCtrlStruct);
}

//Fonction est qui appellée dans un thread, son but est de metter à jour les variables de MinibotCrtlIn et de mettre a jour la vitesse des roues
//en utilisant le controller de vitesse run_speed_controller().
