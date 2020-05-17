///////////////////////////////////////////////////////////////////////
//
// Written by: Matteo Ferretti di Castelferretto and Donatien Doumont
//
//
///////////////////////////////////////////////////////////////////////

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
#include "IO/MCP23017_FPGA/Pinchers_control.hh"
#include "IO/strategy/strategy.h"

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
	P_Struct *my_P_Struct = new P_Struct;
	CAN0_Alternate *can = new CAN0_Alternate(CAN_BR);
	SPI_DE0 *deo;
	deo = new SPI_DE0(0, 125e3);

	init_ctrlStruc(myCtrlStruct);
	init_P_Struct(my_P_Struct);

	SpeedController *spdctrl = new SpeedController(myCtrlStruct, can);
	Odometry *myOdometry = new Odometry(myCtrlStruct);

	spdctrl->init_speed_controller(1);
	spdctrl->set_speed(0, 0);

	delay(2000); //wait for the python UDP client to turn on

	myOdometry->Odometry_init();
	myCtrlStruct->main_states = WAIT_STATE;
	double time_taken;
	int run = 1;
	colorSensorReset();
	reset_dynamixel();

	printf("Welcome to the Poseitron code prototype.\r\n");
	printf("We hope that you will be pleased with the coding and we wish you a great succes.\n\r");

	auto start = std::chrono::steady_clock::now();
	float r, g, b;

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
		case WAIT_STATE:			  // State to let the user some time before the robot starts.
			printf("WAIT_STATE\r\n"); // Time useful to ensure that the speed controller works as expected.

			if (myCtrlStruct->theCtrlIn->t > 15)
			{
				myCtrlStruct->main_states = PNEUMA_TEST_STATE;
				colorSensorReset();
				reset_dynamixel();
				sensorSelect(1);
			}
			break;

		case TEST_PATH_STATE: // State corresponding to the path planning algorithm.
			if (myCtrlStruct->flag_state == 1)
			{
				printf("TEST_PATH_STATE\r\n");
				myCtrlStruct->flag_state = 0;
			}
			main_strategy(myCtrlStruct, my_P_Struct, spdctrl);
			break;

		case PINCHER_DEMO_STATE:			   // State used to validate the good working principle of the pliers and the
			if (myCtrlStruct->flag_state == 1) // color sensors.
			{
				printf("PINCHER_DEMO_STATE\r\n");
				myCtrlStruct->flag_state = 0;
			}
			pincher_demo(myCtrlStruct);
			break;

		case STOP_STATE: // State used to completely stop the motor control card and exit the main loop.
			printf("STOP_STATE\r\n");
			spdctrl->set_speed(0, 0);
			run = 0;
			break;

		case SlAVE_STATE:					   // State used to enter the Slave mode behaviour. It lets Poseitron to be controller by
			if (myCtrlStruct->flag_state == 1) // an UDP client.
			{
				printf("SLAVE_STATE\r\n");
				myCtrlStruct->flag_state = 0;
			}
			break;

		case PNEUMA_TEST_STATE:
			printf("PNEUMA_TEST_STATE\r\n");
			if (myCtrlStruct->theCtrlIn->t - myCtrlStruct->main_t_ref < 5)
			{
			//	printf("Output ON\r\n");
				set_pinchers_output(0b11111111, 0b11111111);
				getRGB(&r, &g, &b);
				printf("R = %f G = %f B = %f\r\n", r, g, b);
			}
			else if (10 > myCtrlStruct->theCtrlIn->t - myCtrlStruct->main_t_ref && myCtrlStruct->theCtrlIn->t - myCtrlStruct->main_t_ref > 5)
			{
				printf("Output OFF\r\n");
				set_pinchers_output(0b00000000, 0b00000000);
			}
			else
			{
				myCtrlStruct->main_t_ref = myCtrlStruct->theCtrlIn->t;
				colorSensorReset();
				sensorSelect(1);
			}
			break;

		default:
			break;
		}
	}

	//********** Liberation de la mémoire  **************

	myOdometry->Odometry_stop();
	spdctrl->Speed_controller_stop();
	free(myCtrlStruct->theUserStruct->theMotRight);
	free(myCtrlStruct->theUserStruct->theMotLeft);
	free(myCtrlStruct->theCtrlIn);
	free(myCtrlStruct->theCtrlOut);
	free(myCtrlStruct->rob_pos);
	free(myCtrlStruct->robot);
	free(myCtrlStruct->theUserStruct);
	free(myCtrlStruct);

	exit(0);
}
