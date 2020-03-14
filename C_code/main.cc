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

using namespace std;

//Constantes utiles
#define CAN_BR 500000
#define CS 0
#define RESETSPI 19

double omega_ref_now_r = 110;
double omega_ref_now_l = 100;

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

	while (true)
	{
		delay(10000);
		//	myCtrlStruct->theCtrlIn->r_wheel_ref = -10;
		//	myCtrlStruct->theCtrlIn->l_wheel_ref = -10;
		printf("wallah je suis la boucle du main !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
		delay(10000);
		//myCtrlStruct->theCtrlIn->r_wheel_ref = omega_ref_now_r;
		//myCtrlStruct->theCtrlIn->l_wheel_ref = omega_ref_now_l;
	}
	free(myCtrlStruct->theCtrlIn);
	free(myCtrlStruct->theCtrlOut);
	free(myCtrlStruct->theUserStruct);
	free(myCtrlStruct);
}

//Fonction est qui appellée dans un thread, son but est de metter à jour les variables de MinibotCrtlIn et de mettre a jour la vitesse des roues
//en utilisant le controller de vitesse run_speed_controller().
