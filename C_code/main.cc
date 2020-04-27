#include <cstdio>
#include <stdio.h>

#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"
#include "IO/COM/TCS3472_I2C/TCS3472_FPGA.hh"
#include "IO/Dynamixel/DynamixelFunctions.h"
#include "IO/Dynamixel/MyDynamixel.h"
#include "IO/Pincher_Demo/Pincher_Demo.hh"
#include "IO/Calibration/odo_calibration.h"
//#include <python3.7/Python.h>
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

	system("python /home/pi/Poseitron-Code/Python_code/slider_PID.py &");

	delay(5000); //wait for the python code to generate some files

	SpeedController *spdctrl = new SpeedController(myCtrlStruct, can);
	Odometry *myOdometry = new Odometry(myCtrlStruct);

	spdctrl->init_speed_controller(1);
	spdctrl->set_speed(0, 0);
	spdctrl->Speed_controller_stop();
	myOdometry->Odometry_init();
	myCtrlStruct->main_states = WAIT_STATE;
	double time_taken;
	int run = 1;
	colorSensorReset();
	reset_dynamixel();

	printf("Welcome to the Poseitron code prototype.\r\n");
	printf("We hope that you will be pleased with the coding and we wish you a great succes.\n\r");

	auto start = std::chrono::steady_clock::now();

	//********  Début du comportement du robot **********

	while (run)
	{
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
		time_taken = (elapsed.count());
		myCtrlStruct->theCtrlIn->t = time_taken / 1000.0;
		spdctrl->updateLowCtrl();
		myOdometry->Odometry_update();

		switch (myCtrlStruct->main_states)
		{
		case WAIT_STATE:
			printf("WAIT_STATE\r\n");

			if (myCtrlStruct->theCtrlIn->t > 5)
			{
				myCtrlStruct->main_states = ODO_CALIB_STATE;
				colorSensorReset();
				reset_dynamixel();
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

		case PINCHER_DEMO_STATE:
			printf("PINCHER_DEMO_STATE\r\n");
			pincher_demo(myCtrlStruct);
			break;

		case ODO_CALIB_STATE:
			printf("ODO_CALIB_STATE\r\n");
			odo_calibration(myCtrlStruct, spdctrl, myOdometry);
			break;

		case STOP_STATE:
			printf("STOP_STATE\r\n");
			//	printf("Left = %f Right = %f\r\n", myCtrlStruct->stopvalues[0], myCtrlStruct->stopvalues[1]);
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
