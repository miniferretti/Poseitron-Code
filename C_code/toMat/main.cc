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

	P_Struct *my_P_Struct = new P_Struct; 

	delay(100);

	init_ctrlStruc(myCtrlStruct);
	inti_P_Struct(my_P_Struct);

	SpeedController *spctrl = new SpeedController(myCtrlStruct, can);

	myCtrlStruct->theCtrlIn->r_wheel_ref = omega_ref_now_r;
	myCtrlStruct->theCtrlIn->l_wheel_ref = omega_ref_now_l;

	spctrl->init_speed_controller(1);

	myOdometry->Odometry_init();
	myCtrlStruct->main_states = SlAVE_STATE;
	double time_taken;
	int run = 1;
	colorSensorReset();
	reset_dynamixel();

	printf("Welcome to the Poseitron code prototype.\r\n");
	printf("We hope that you will be pleased with the coding and we wish you a great succes.\n\r");

	//********  Début du comportement du robot **********

	while (true)
	{
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
		time_taken = (elapsed.count());
		myCtrlStruct->theCtrlIn->t = time_taken / 1000.0;
		spdctrl->updateLowCtrl();
		myOdometry->Odometry_update();

		myCtrlStruct->main_t_ref = myCtrlStruct->theCtrlIn->t;

		switch (myCtrlStruct->main_states)
		{
		case WAIT_STATE:
			printf("WAIT_STATE\r\n");

			if (myCtrlStruct->theCtrlIn->t > 5)
			{
				myCtrlStruct->main_states = TEST_PATH_STATE;
				colorSensorReset();
				reset_dynamixel();
			}
			break;

		case CALIB_STATE:
			printf("CALIB_STATE\r\n");
			calibration(myCtrlStruct, spdctrl, myOdometry);
			break;

		case TEST_PATH_STATE:
			printf("TEST_PATH_STATE\r\n");
			main_strategy(myCtrlStruct, my_P_Struct);
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

		case SlAVE_STATE:
			printf("SLAVE_STATE\r\n");
			break;

		default:
			break;
		}
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
