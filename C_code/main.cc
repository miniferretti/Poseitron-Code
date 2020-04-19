#include <cstdio>
#include <stdio.h>

#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"
#include "IO/COM/TCS3472_I2C/TCS3472_FPGA.hh"
#include "IO/Dynamixel/DynamixelFunctions.h"
#include "IO/Dynamixel/MyDynamixel.h"
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
#include <chrono>
#include "IO/Calibration/Calibration.hh"
#include "IO/Mid_level_controller/Avoid150.hh"

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
	wiringPiSetup();
	CtrlStruct *myCtrlStruct = new CtrlStruct;
	CAN0_Alternate *can = new CAN0_Alternate(CAN_BR);
	SPI_DE0 *deo;
	deo = new SPI_DE0(0, 125e3);
	init_ctrlStruc(myCtrlStruct);

	SpeedController *spdctrl = new SpeedController(myCtrlStruct, can);
	Odometry *myOdometry = new Odometry(myCtrlStruct);
	sensorSelect(2);

	spdctrl->init_speed_controller(1);
	spdctrl->set_speed(0, 0);
	myOdometry->Odometry_init();
	myCtrlStruct->main_states = WAIT_STATE;
	auto start = std::chrono::steady_clock::now();
	double time_taken;
	int run = 1;
	colorSensorReset();
	reset_dynamixel();
	sensorSelect(1);
	float r, g, b, c;

	printf("Welcome to the Poseitron code prototype.\r\n");
	printf("We hope that you will be pleased with the coding and we wish you a great succes.\n\r");

	//********  Début du comportement du robot **********

	while (run)
	{
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
		time_taken = (elapsed.count());
		myCtrlStruct->theCtrlIn->t = time_taken / 1000.0;
		spdctrl->updateLowCtrl();
		myOdometry->Odometry_update();

		getRGB(&r, &g, &b);

		for (int i = 0; i <= 255; i++)
		{
			if(DynLightLed(0x10)){
				break;
			}
		}
		break;

		printf("red = %f green = %f blue = %f \r\n", r, g, b); // tésté pour verifier que les senseurs de couleur focntionnent

		switch (myCtrlStruct->main_states)
		{
		case WAIT_STATE:
			//printf("WAIT_STATE\r\n");

			if (myCtrlStruct->theCtrlIn->t > 10)
			{
				myCtrlStruct->main_states = WAIT_STATE;
				myCtrlStruct->calib_states = CALIB_1;
				sensorSelect(0);
			}
			break;

		case CALIB_STATE:
			printf("CALIB_STATE\r\n");
			calibration(myCtrlStruct, spdctrl, myOdometry);
			break;

		case AVOID150_STATE:
			printf("AVOID150_STATE\r\n");
			avoid150(myCtrlStruct, spdctrl, myOdometry);
			break;

		case STOP_STATE:
			printf("STOP_STATE\r\n");
			printf("Left = %f Right = %f\r\n", myCtrlStruct->stopvalues[0], myCtrlStruct->stopvalues[1]);
			spdctrl->set_speed(0, 0);
			run = 0;
			break;

		default:
			break;
		}
	}

	spdctrl->set_speed(0, 0);

	//********** Liberation de la mémoire  **************

	myOdometry->Odometry_stop();
	spdctrl->Speed_controller_stop();
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
