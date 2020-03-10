#include <math.h>
#include "CtrlStruct.h"

void beacon_following(CtrlStruct *theCtrlStruct)
{
	// get both reference speed and combine them
	// convention :
	// right wheel ref is sum and left wheel ref is difference
	double omega_ref_dist = theCtrlStruct->theUserStruct->omega_ref_dist;
	double omega_ref_dir = theCtrlStruct->theUserStruct->omega_ref_dir;

	theCtrlStruct->theUserStruct->omega_ref[R_ID] = omega_ref_dist + omega_ref_dir;
	theCtrlStruct->theUserStruct->omega_ref[L_ID] = omega_ref_dist - omega_ref_dir;
}

void beacon_distance_controller(CtrlStruct *theCtrlStruct)
{
	// get the parameters from beacon and robot
	double beacon_distance = theCtrlStruct->theUserStruct->beacon_distance;
	double wheel_radius = theCtrlStruct->theUserStruct->wheel_radius;

	// get the parameters for the controller
	double Kp = theCtrlStruct->theUserStruct->Kp2;
	double Ki = theCtrlStruct->theUserStruct->Ki2;
	double t_step = theCtrlStruct->theCtrlIn->t - theCtrlStruct->theUserStruct->t_last_dist;
	theCtrlStruct->theUserStruct->t_last_dist = theCtrlStruct->theCtrlIn->t;

	// give the reference parameters
	double beacon_distance_ref = theCtrlStruct->theUserStruct->beacon_distance_ref;
	double error = (beacon_distance_ref - beacon_distance);

	// controller on the robot global speed
	double term1 = Kp * error;
	theCtrlStruct->theUserStruct->term2_dist = theCtrlStruct->theUserStruct->term2_dist + Ki * error * t_step;
	double speed_ref_dist = term1 + theCtrlStruct->theUserStruct->term2_dist;

	double omega_ref_dist = speed_ref_dist / wheel_radius;
}

void beacon_direction_controller(CtrlStruct *theCtrlStruct)
{
	//get the parameters for beacon
	double beacon_direction = theCtrlStruct->theUserStruct->beacon_direction;
	double wheel_radius = theCtrlStruct->theUserStruct->wheel_radius;

	// get the parameters for the controller
	double Kp = theCtrlStruct->theUserStruct->Kp3;
	double Ki = theCtrlStruct->theUserStruct->Ki3;
	double t_step = theCtrlStruct->theCtrlIn->t - theCtrlStruct->theUserStruct->t_last_dir;
	theCtrlStruct->theUserStruct->t_last_dir = theCtrlStruct->theCtrlIn->t;

	// give the reference parameters
	double beacon_direction_ref = theCtrlStruct->theUserStruct->beacon_direction_ref;
	double error = (beacon_direction_ref - beacon_direction);

	// controlleur on the robot global speed
	double term1 = Kp * error;
	theCtrlStruct->theUserStruct->term2_dir = theCtrlStruct->theUserStruct->term2_dir + Ki * error * t_step;
	double speed_ref_dir = term1 + theCtrlStruct->theUserStruct->term2_dir;

	double omega_ref_dir = speed_ref_dir / wheel_radius;
}
