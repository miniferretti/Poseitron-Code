#include <cstdio>
#include <stdio.h>

#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"

#include <iostream>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include "IO/Speed_Controller/SpeedController.hh"
#include "IO/Odometry/Odometry.hh"
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <string>
#include <sstream>
#include "IO/COM/TCS3472_I2C/TCS3472_I2C.hh"
#include <chrono>
#include "IO/Calibration/Calibration.hh"

using namespace std;

//Constantes utiles
#define CAN_BR 500000
#define CS 0
#define RESETSPI 19

double omega_ref_now_r[6] = {0, 50, 50, -50, -50, 0};
double omega_ref_now_l[6] = {0, 50, 50, 50, -50, 0};
int l = 6;

int main()
{
	CtrlStruct *myCtrlStruct = new CtrlStruct;
	CAN0_Alternate *can = new CAN0_Alternate(CAN_BR);
	SPI_DE0 *deo;
	deo = new SPI_DE0(0, 125e3);
	pthread_mutex_t theMutex = PTHREAD_MUTEX_INITIALIZER;
	init_ctrlStruc(myCtrlStruct);

	SpeedController *spdctrl = new SpeedController(myCtrlStruct, can, &theMutex);
	Odometry *myOdometry = new Odometry(myCtrlStruct, &theMutex);
	Adafruit_TCS34725 *myColorSensor = new Adafruit_TCS34725();

	spdctrl->set_speed(0, 0);
	spdctrl->init_speed_controller(1);
	spdctrl->run_speed_controller();
	myOdometry->Odometry_init();
	myOdometry->Odometry_start();
	myCtrlStruct->main_states = WAIT_STATE;

	printf("Welcome to the Poseitron code prototype.\r\n");
	printf("We hope that you will be pleased with the coding and we wish you a great succes.\n\r");

	//********  Début du comportement du robot **********

	while (myCtrlStruct->main_states != STOP_STATE)
	{
		switch (myCtrlStruct->main_states)
		{
		case WAIT_STATE:
			printf("C'est la WAIT_STATE\r\n");
			spdctrl->set_speed(0, 0);
			if (myCtrlStruct->theCtrlIn->t > 30)
			{
				myCtrlStruct->main_states = CALIB_STATE;
			}
			break;

		case CALIB_STATE:
			printf("C'est la CALIB_STATE\r\n");
			myCtrlStruct->calib_states = CALIB_1;
			calibration(myCtrlStruct, spdctrl);
			break;

		case STOP_STATE:
			printf("STOP_STATE\r\n");
			spdctrl->set_speed(0, 0);
			break;

		default:
			break;
		}
	}

	spdctrl->set_speed(0, 0);

	//********** Liberation de la mémoire  **************

	myOdometry->Odometry_stop();
	spdctrl->speed_controller_active(0);
	free(myCtrlStruct->theUserStruct->theMotRight);
	free(myCtrlStruct->theUserStruct->theMotLeft);
	free(myCtrlStruct->theCtrlIn);
	free(myCtrlStruct->theCtrlOut);
	free(myCtrlStruct->theUserStruct);
	free(myCtrlStruct->rob_pos);
	free(myCtrlStruct->robot);
	free(myCtrlStruct);
	exit(0);
}
