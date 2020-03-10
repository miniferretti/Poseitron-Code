#include <cstdio>
#include <stdio.h>
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"
#include "CtrlStruct.h"
#include <iostream>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include "IO/ctrl_io.h"
//#include <chrono>
#include <pthread.h>
//#include <functional>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <string>
#include <sstream>
#include "IO/COM/TCS3472_I2C/TCS3472_I2C.hh"
//#include <encapsulated code by tasks>
#include <speed_controller.h>

using namespace std;

//Constantes utiles
#define CAN_BR 125e3
#define CS 0
#define RESETSPI 19

CtrlStruct *myCtrlStruct = new CtrlStruct;

//CtrlStruct *myCtrlStruct ;

//Constant values for the updateCrtlIn() routine
//paramètre de la conversion omega->vitesse pour les roues
double TicsRoue = 28000;
double samplingDE0 = 500;
double Rroue = 0.03; // Valeur en m

double omega_roue_ref_l[6] = {0, 6.2832, 0, 6.2832, 0, 6.2832};
double omega_roue_ref_r[6] = {0, 6.2832, 0, -6.2832, 0, 6.2832};

double omega_ref_now_r = 0.0;
double omega_ref_now_l = 0.0;
double dt_ref = 3;

//Declaration des fonctions
void *updateCrtlIn(void *);
void run_speed_controller(CtrlStruct *theCtrlStruct);
void init_speed_controller(CtrlStruct *theCtrlStruct);
int saturation(double upperLimit, double lowerLimit, double *u);

int main()
{
	//CtrlStruct *myCtrlStruct ;
	myCtrlStruct->theUserStruct = new UserStruct;
	myCtrlStruct->theCtrlIn = new CtrlIn;
	myCtrlStruct->theCtrlOut = new CtrlOut;

	printf("Welcome to the Poseitron code prototype.\r\n");
	printf("We hope that you will be pleased with the coding and we wish you a great succes.\n\r");

	//test the motor control
	CAN *can;
	can = new CAN(CAN_BR);
	SPI_DE0 *deo;
	deo = new SPI_DE0(0, 125e3);
	delay(100);

	can->configure();
	delay(100);

	can->ctrl_motor(1);
	can->check_receive();

	//	can->push_PropDC(0, 0);
	can->check_receive();

	init_speed_controller(myCtrlStruct);

	//Creation du thread pour la fonction updateCrtlIn
	pthread_t t;
	pthread_create(&t, NULL, &updateCrtlIn, can);

	//********  Début du comportement du robot **********
	int i = 0;
	clock_t time_rec;
	time_rec = clock();
	while (true)
	{
		/*if ( (long double) (time_rec - clock())/CLOCKS_PER_SEC >= dt_ref && i<6){
			printf("time_rec = %f & omega_ref_now_l = %f \r \n", (double) (time_rec - clock()), omega_ref_now_l);
			omega_ref_now_l = omega_roue_ref_l[i];
			omega_ref_now_r = omega_roue_ref_r[i];
			time_rec = clock(); 
			i++;
		}*/
		//printf("%s\n\r", theUserStruct.beacon_detect ? "true" : "false");
		//printf("The distance to the beacon is %f\n\r",theUserStruct.beacon_distance);
		//getBeaconAngleAndDist(MinibotCrtlIn.last_rising_pos,MinibotCrtlIn.last_falling_pos);
		//printf("La distance est %f \r\n",theUserStruct.beacon_distance);

		//can->ctrl_led(1);
		delay(100);
		//can->ctrl_led(0);
	}
	free(myCtrlStruct);
}

//Fonction est qui appellée dans un thread, son but est de metter à jour les variables de MinibotCrtlIn et de mettre a jour la vitesse des roues
//en utilisant le controller de vitesse run_speed_controller().
void *updateCrtlIn(void *theCani)
{

	CAN *theCan = (CAN *)theCani;
	unsigned char buffer[5] = {0};
	clock_t t;
	t = clock();

	while (true)
	{
		//adresse des roues
		buffer[0] = 0x00;
		buffer[1] = 0x00;
		buffer[2] = 0x00;
		buffer[3] = 0x00;
		buffer[4] = 0x00;

		wiringPiSPIDataRW(0, buffer, 5);
		delay(0.5);

		myCtrlStruct->theCtrlIn->r_wheel_speed = ((double)(int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4])) * samplingDE0 * 2 * M_PI / TicsRoue;
		myCtrlStruct->theCtrlIn->l_wheel_speed = ((double)(int16_t)((uint16_t)buffer[1] << 8 | (uint16_t)buffer[2])) * samplingDE0 * 2 * M_PI / TicsRoue;

		//printf("La vitesse de la roue est de %f ou %f\r\n",MinibotCrtlIn.l_wheel_speed,MinibotCrtlIn.r_wheel_speed);
		//	printf("%d %d %d %d\r\n",buffer[1],buffer[2],buffer[3],buffer[4]);

		//printf("%f et %f\r\n",((double)(int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4])),((double)(int16_t)((uint16_t)buffer[1] << 8 | (uint16_t)buffer[2])));

		printf("%f %f \r\n", myCtrlStruct->theCtrlIn->r_wheel_speed, myCtrlStruct->theCtrlIn->l_wheel_speed);

		run_speed_controller(myCtrlStruct);

		theCan->push_PropDC(myCtrlStruct->theCtrlOut->wheel_commands[L_ID], myCtrlStruct->theCtrlOut->wheel_commands[R_ID]);

		//Mise a jour du pas de temps
		t = clock() - t;
		double time_taken = ((double)t) / CLOCKS_PER_SEC;
		myCtrlStruct->theCtrlIn->t = time_taken; //Temps utilisé pour mettre a jour les valeurs et appeler le speed controller
	}
}


