#include <cstdio>
#include <stdio.h>
//#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/CAN/CAN_Alternate.hh"
//#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"
//#include "IO/Speed_Controller/CtrlStruct.hh"
//#include "IO/Speed_Controller/ctrl_io.h"
#include <iostream>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include "IO/Speed_Controller/SpeedController.hh"
//#include <chrono>
#include <pthread.h>
//#include <functional>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <string>
#include <sstream>
#include "IO/COM/TCS3472_I2C/TCS3472_I2C.hh"
#include <chrono>

using namespace std;

//Constantes utiles
#define CAN_BR 500e3
#define CS 0
#define RESETSPI 19

CtrlStruct *myCtrlStruct = new CtrlStruct;

//Constant values for the updateCrtlIn() routine
//paramètre de la conversion omega->vitesse pour les roues

double omega_ref_now_r = 10;
double omega_ref_now_l = 10;
double dt_ref = 3;

//Declaration des fonctions
void *updateCrtlIn(void *unused);
double get_speed(char a1, char a2);
int get_int(char a1, char a2);

int main()
{

	init_ctrlStruc(myCtrlStruct);
	myCtrlStruct->theCtrlIn->r_wheel_ref = omega_ref_now_r;
	myCtrlStruct->theCtrlIn->l_wheel_ref = omega_ref_now_l;

	printf("Welcome to the Poseitron code prototype.\r\n");
	printf("We hope that you will be pleased with the coding and we wish you a great succes.\n\r");

	SPI_DE0 *deo;
	deo = new SPI_DE0(0, 125e3);
	delay(100);

	CAN0configure(CAN_BR);
	delay(100);
	CAN0ctrl_motor(1);
	init_speed_controller(myCtrlStruct);

	//Creation du thread pour la fonction updateCrtlIn
	pthread_t t;
	pthread_create(&t, NULL, &updateCrtlIn, NULL);

	//********  Début du comportement du robot **********
	int i = 0;
	clock_t time_rec;
	time_rec = clock();
	while (true)
	{

		CAN0ctrl_led(0);
		delay(100);
		CAN0ctrl_led(1);
	}
	free(myCtrlStruct->theCtrlIn);
	free(myCtrlStruct->theCtrlOut);
	free(myCtrlStruct->theUserStruct);
	free(myCtrlStruct);
}

//Fonction est qui appellée dans un thread, son but est de metter à jour les variables de MinibotCrtlIn et de mettre a jour la vitesse des roues
//en utilisant le controller de vitesse run_speed_controller().
void *updateCrtlIn(void *unused)
{

	unsigned char buffer[5] = {0}; // create timers.

	// get current time.
	auto start = std::chrono::steady_clock::now();

	while (true)
	{

		//adresse des roues
		buffer[0] = 0x00;
		buffer[1] = 0x00;
		buffer[2] = 0x00;
		buffer[3] = 0x00;
		buffer[4] = 0x00;

		wiringPiSPIDataRW(0, buffer, 5);
		delay(100);

		myCtrlStruct->theCtrlIn->l_wheel_speed = (((double)(int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4])) * myCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (7 * myCtrlStruct->theUserStruct->tics);
		myCtrlStruct->theCtrlIn->r_wheel_speed = (((double)(int16_t)((uint16_t)buffer[1] << 8 | (uint16_t)buffer[2])) * myCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (7 * myCtrlStruct->theUserStruct->tics);

		//myCtrlStruct->theCtrlIn->l_wheel_speed = get_speed(buffer[3], buffer[4]);
		//myCtrlStruct->theCtrlIn->r_wheel_speed = get_speed(buffer[1], buffer[2]);

		printf(" l_wheel_speed %f", myCtrlStruct->theCtrlIn->l_wheel_speed);
		printf(" r_wheel_speed %f\n", myCtrlStruct->theCtrlIn->r_wheel_speed);

		run_speed_controller(myCtrlStruct);

		printf(" commande gauche %f", myCtrlStruct->theCtrlOut->wheel_commands[L_ID]);
		printf(" commande droite %f\n", myCtrlStruct->theCtrlOut->wheel_commands[R_ID]);

		CAN0pushPropDC(myCtrlStruct->theCtrlOut->wheel_commands[L_ID], myCtrlStruct->theCtrlOut->wheel_commands[R_ID]);

		//Mise a jour du pas de temps

		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
		double time_taken = (elapsed.count());
		printf("time taken: %f\r\n", time_taken / 1000);
		myCtrlStruct->theCtrlIn->t = time_taken / 1000; //Temps utilisé pour mettre a jour les valeurs et appeler le speed controller
	}
}

double get_speed(char a1, char a2)
{
	double ratio = 14 * 4;
	int val = get_int(a1, a2);

	double max = 512;
	double T = 1.0 / 2000;

	return (2 * M_PI * ((double)val) / (max * T * ratio));
}

int get_int(char a1, char a2)
{
	int neg = (int)a1 / 128 == 1;

	int c = (int)a1 * 256 + (int)a2;

	if (neg)
	{
		c = -(65536 - c + 1);
	}

	return c;
}
