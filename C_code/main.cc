#include <cstdio>
#include <stdio.h>
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"
#include "CtrlStruct.h"
#include <iostream>
#include <wiringPiSPI.h>
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
using namespace std;

//Constantes utiles
#define CAN_BR 125e3
#define CS 0
#define RESETSPI 19

//global Controle structures
struct CtrlIn MinibotCrtlIn;
struct CtrlOut MinibotCrtlOut;
struct UserStruct theUserStruct;

//PID values for the motor controller
double Kp1 = 0.045;  //valeur Dona Nico et Aurèle 3.51e-2;
double Ki1 = 0.2071; //valeur Dona Nico et Aurèle 1.982e-1;

//PID values for the Tower if needed
double Kp2 = 0;
double Ki2 = 0;

//Speed references declaration
double omega_refR = 0;
double omega_refL = 0;

//Constant values for the updateCrtlIn() routine
//paramètre de la conversion omega->vitesse pour les roues
double TicsRoue = 28000;
double samplingDE0 = 500;
double Rroue = 0.03; // Valeur en m

//Declaration des fonctions
void *updateCrtlIn(void *);
void run_speed_controller(double omega_refLi, double omega_refRi);

int main()
{
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

	can->push_PropDC(0, 0);
	can->check_receive();

	//Creation du thread pour la fonction updateCrtlIn
	pthread_t t;
	pthread_create(&t, NULL, &updateCrtlIn, can);

	//********  Début du comportement du robot **********


	

	while (true)
	{
		//printf("%s\n\r", theUserStruct.beacon_detect ? "true" : "false");
		//printf("The distance to the beacon is %f\n\r",theUserStruct.beacon_distance);
		//getBeaconAngleAndDist(MinibotCrtlIn.last_rising_pos,MinibotCrtlIn.last_falling_pos);
		//printf("La distance est %f \r\n",theUserStruct.beacon_distance);

		//can->ctrl_led(1);
		delay(100);
		//can->ctrl_led(0);
	}
}

//Fonction est qui appellée dans un thread, son but est de metter à jour les variables de MinibotCrtlIn et de mettre a jour la vitesse des roues
//en utilisant le controller de vitesse run_speed_controller().
void *updateCrtlIn(void *theCani)
{
	CAN *theCan = (CAN *)theCani;

	unsigned char buffer[5] = {0};

	while (true)
	{

		clock_t t;
		t = clock();
		delay(0);

		//adresse des roues
		buffer[0] = 0x00;
		buffer[1] = 0x00;
		buffer[2] = 0x00;
		buffer[3] = 0x00;
		buffer[4] = 0x00;

		wiringPiSPIDataRW(0, buffer, 5);

		MinibotCrtlIn.r_wheel_speed = ((double)(int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4])) * samplingDE0 * 2 * M_PI / TicsRoue;
		MinibotCrtlIn.l_wheel_speed = ((double)(int16_t)((uint16_t)buffer[1] << 8 | (uint16_t)buffer[2])) * samplingDE0 * 2 * M_PI / TicsRoue;

		//printf("La vitesse de la roue est de %f ou %f\r\n",MinibotCrtlIn.l_wheel_speed,MinibotCrtlIn.r_wheel_speed);
		//	printf("%d %d %d %d\r\n",buffer[1],buffer[2],buffer[3],buffer[4]);

		//printf("%f et %f\r\n",((double)(int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4])),((double)(int16_t)((uint16_t)buffer[1] << 8 | (uint16_t)buffer[2])));

		printf("%f %f \r\n", MinibotCrtlIn.r_wheel_speed, MinibotCrtlIn.l_wheel_speed);

		run_speed_controller(omega_refL,omega_refR);

		theCan->push_PropDC(MinibotCrtlOut.wheel_commands[0],MinibotCrtlOut.wheel_commands[1]);
		
		//Mise a jour du pas de temps
		t = clock() - t;
		double time_taken = ((double)t) / CLOCKS_PER_SEC;
		MinibotCrtlIn.t = time_taken; //Temps utilisé pour mettre a jour les valeurs et appeler le speed controller
	}
}

//Il faut adapter le controller pour qu'il puisse utiliser ce qu'il y a ici.
void run_speed_controller(double omega_refLi, double omega_refRi)
{
	double omega_ref_lwheel = omega_refLi;
	double omega_ref_rwheel = omega_refRi;
	double omega_real_lwheel = MinibotCrtlIn.l_wheel_speed;
	double omega_real_rwheel = MinibotCrtlIn.r_wheel_speed;
	double t_step = MinibotCrtlIn.t;

	//taking the gearbox in account
	double omega_ref_l = 14 * omega_ref_lwheel;
	double omega_ref_r = 14 * omega_ref_rwheel;
	double omega_real_l = 14 * omega_real_lwheel;
	double omega_real_r = 14 * omega_real_rwheel;

	printf("up\r\n");

	//controller for left wheel
	double term1_l = Kp1 * (omega_ref_l - omega_real_l);
	theUserStruct.term2_l = theUserStruct.used_term2_l + Ki1 * (omega_ref_l - omega_real_l) * t_step;
	double th_uaref_l = term1_l + theUserStruct.term2_l;
	//controller for right wheel
	double term1_r = Kp1 * (omega_ref_r - omega_real_r);
	theUserStruct.term2_r = theUserStruct.used_term2_r + Ki1 * (omega_ref_r - omega_real_r) * t_step;
	double th_uaref_r = term1_r + theUserStruct.term2_r;

	//conversion to [-100:100] range
	double max_uaref = 24 * 0.95;

	//saturation checks
	if (th_uaref_r < -max_uaref)
	{
		MinibotCrtlOut.wheel_commands[0] = -100;
	}
	if (th_uaref_l < -max_uaref)
	{
		MinibotCrtlOut.wheel_commands[1] = -100;
	}

	if (th_uaref_r > max_uaref)
	{
		MinibotCrtlOut.wheel_commands[0] = 100;
	}
	if (th_uaref_l > max_uaref)
	{
		MinibotCrtlOut.wheel_commands[1] = 100;
	}

	else
	{
		MinibotCrtlOut.wheel_commands[0] = th_uaref_r * 100 / (24 * 0.95);
		theUserStruct.used_term2_r = theUserStruct.term2_r;
		MinibotCrtlOut.wheel_commands[1] = th_uaref_l * 100 / (24 * 0.95);
		theUserStruct.used_term2_l = theUserStruct.term2_l;
	}

	return;
}

