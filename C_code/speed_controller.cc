#include <speed_controller.h>
#include "CtrlStruct.h"

void run_speed_controller(CtrlStruct *theCtrlStruct)
{

	double omega_ref_l = omega_ref_now_l * theCtrlStruct->theUserStruct->ratio;
	double omega_ref_r = omega_ref_now_r * theCtrlStruct->theUserStruct->ratio;
	double r_wheel_speed = theCtrlStruct->theCtrlIn->r_wheel_speed * 14;
	double l_wheel_speed = theCtrlStruct->theCtrlIn->l_wheel_speed * 14;
	double e_l = omega_ref_l - l_wheel_speed;
	double e_r = omega_ref_r - r_wheel_speed;
	double upperCurrentLimit = theCtrlStruct->theUserStruct->upperCurrentLimit;
	double lowerCurrentLimit = theCtrlStruct->theUserStruct->lowerCurrentLimit;
	double kphi = theCtrlStruct->theUserStruct->kphi;
	double Kp = theCtrlStruct->theUserStruct->Kp;
	double Ki = theCtrlStruct->theUserStruct->Ki;
	double t = theCtrlStruct->theCtrlIn->t;
	double tp = theCtrlStruct->theUserStruct->t_p;
	double Ra = theCtrlStruct->theUserStruct->Ra;
	double dt = t - tp;

	double u_l = Kp * e_l;
	double u_r = Kp * e_r;
	double i_e_l = theCtrlStruct->theUserStruct->i_e_l;
	double i_e_r = theCtrlStruct->theUserStruct->i_e_r;

	//LEFT WHEEL
	if (!theCtrlStruct->theUserStruct->sat_l)
	{
		//The integral action is only done if there is no saturation of current.
		i_e_l += dt * e_l;
	}
	u_l += i_e_l * Ki;
	u_l += kphi * l_wheel_speed;
	theCtrlStruct->theUserStruct->sat_l = saturation(theCtrlStruct->theUserStruct->upperCurrentLimit + kphi * l_wheel_speed, theCtrlStruct->theUserStruct->lowerCurrentLimit - kphi * l_wheel_speed, &u_l);

	//RIGHT WHEEL
	if (!theCtrlStruct->theUserStruct->sat_r)
	{
		//The integral action is only done if there is no saturation of current.
		i_e_r += dt * e_r;
	}
	u_r += i_e_r * Ki;
	u_r += kphi * r_wheel_speed;
	theCtrlStruct->theUserStruct->sat_r = saturation(theCtrlStruct->theUserStruct->upperCurrentLimit + kphi * r_wheel_speed, theCtrlStruct->theUserStruct->lowerCurrentLimit - kphi * r_wheel_speed, &u_r);

	//OUTPUT
	theCtrlStruct->theCtrlOut->wheel_commands[L_ID] = u_l * 100 / (theCtrlStruct->theUserStruct->upperVoltageLimit);
	theCtrlStruct->theCtrlOut->wheel_commands[R_ID] = u_r * 100 / (theCtrlStruct->theUserStruct->upperVoltageLimit);
	theCtrlStruct->theUserStruct->t_p = t;
	theCtrlStruct->theUserStruct->i_e_l = i_e_l;
	theCtrlStruct->theUserStruct->i_e_r = i_e_r;
}
//Loading of the variable
void init_speed_controller(CtrlStruct *theCtrlStruct)
{

	double Ra = 5.84;
	double Kv = 4.3e-5;
	double J_rotor = 12e-7;
	double J_robot = 3 * 7.03125e-6;
	double J = J_rotor + J_robot;
	double tau_m = J / Kv;
	double kphi = 37.83e-3;
	double Kp = 3 * Ra * Kv / kphi;
	double Ki = Kp * ((Ra * Kv + kphi * Kp) / (J * Ra) - 3 / tau_m);
	double Current_max = 0.82; // Ampere
	double secu = 0.95;

	theCtrlStruct->theUserStruct->ratio = 7;
	theCtrlStruct->theUserStruct->kphi = kphi;
	theCtrlStruct->theUserStruct->Ra = Ra;
	theCtrlStruct->theUserStruct->Ki = Ki;
	theCtrlStruct->theUserStruct->Kp = Kp;
	theCtrlStruct->theUserStruct->t_p = 0.0;
	theCtrlStruct->theUserStruct->upperCurrentLimit = Ra * Current_max;
	theCtrlStruct->theUserStruct->lowerCurrentLimit = -Ra * Current_max;
	theCtrlStruct->theUserStruct->upperVoltageLimit = 24 * secu;
	theCtrlStruct->theUserStruct->lowerVoltageLimit = -24 * secu;
	theCtrlStruct->theUserStruct->i_e_l = 0.0;
	theCtrlStruct->theUserStruct->i_e_r = 0.0;
	theCtrlStruct->theUserStruct->sat_l = 0;
	theCtrlStruct->theUserStruct->sat_r = 0;
}

int saturation(double upperLimit, double lowerLimit, double *u)
{
	if (*u > upperLimit)
	{
		*u = upperLimit;
		return 1;
	}
	else if (*u < lowerLimit)
	{
		*u = lowerLimit;
		return 1;
	}
	else
		return 0;
}