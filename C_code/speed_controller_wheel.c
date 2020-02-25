#include <stdio.h>
#include <stdlib.h>
#include "CtrlStruct.h"

void run_speed_controller(CtrlStruct* theCtrlStruct, double* omega_ref){
	double omega_ref_lwheel = omega_ref[L_ID];
	double omega_ref_rwheel = omega_ref[R_ID];
	double omega_real_lwheel = theCtrlStruct->theCtrlIn->l_wheel_speed;
	double omega_real_rwheel = theCtrlStruct->theCtrlIn->r_wheel_speed;
	double t_step = theCtrlStruct->theCtrlIn->t - theCtrlStruct->theUserStruct->t_last;
	theCtrlStruct->theUserStruct->t_last = theCtrlStruct->theCtrlIn->t;
	double Kp1 = theCtrlStruct->theUserStruct->Kp1;
	double Ki1 = theCtrlStruct->theUserStruct->Ki1;
	int ratio = theCtrlStruct->theUserStruct->ratio; 


 
	//taking the gearbox in account 
	double omega_ref_l = ratio*omega_ref_lwheel;
	double omega_ref_r = ratio*omega_ref_rwheel;
	double omega_real_l = ratio*omega_real_lwheel;
	double omega_real_r = ratio*omega_real_rwheel;

	//controller for left wheel 
	double term1_l = Kp1 * (omega_ref_l - omega_real_l);
	theCtrlStruct->theUserStruct->term2_l = theCtrlStruct->theUserStruct->used_term2_l + Ki1 * (omega_ref_l - omega_real_l) * t_step;
	double th_uaref_l = term1_l + theCtrlStruct->theUserStruct->term2_l;
	//controller for right wheel 
	double term1_r = Kp1 * (omega_ref_r - omega_real_r);
	theCtrlStruct->theUserStruct->term2_r = theCtrlStruct->theUserStruct->used_term2_r + Ki1 * (omega_ref_r - omega_real_r) * t_step;
	double th_uaref_r = term1_r + theCtrlStruct->theUserStruct->term2_r;

	//conversion to [-100:100] range
	double max_uaref = 24*0.95;

	//saturation checks 
	if (th_uaref_r < -max_uaref) { theCtrlStruct->theCtrlOut->wheel_commands[0] = -100; }
	if (th_uaref_l < -max_uaref) { theCtrlStruct->theCtrlOut->wheel_commands[1] = -100; }

	if (th_uaref_r > max_uaref) { theCtrlStruct->theCtrlOut->wheel_commands[0] = 100; }
	if (th_uaref_l > max_uaref) { theCtrlStruct->theCtrlOut->wheel_commands[1] = 100; }

	else
	{
		theCtrlStruct->theCtrlOut->wheel_commands[0] = th_uaref_r*100/(24*0.95);
		theCtrlStruct->theUserStruct->used_term2_r = theCtrlStruct->theUserStruct->term2_r;
		theCtrlStruct->theCtrlOut->wheel_commands[1] = th_uaref_l*100/(24*0.95);
		theCtrlStruct->theUserStruct->used_term2_l = theCtrlStruct->theUserStruct->term2_l;
	}

	return;
}

void init_speed_controller(CtrlStruct* theCtrlStruct){
	//theCtrlStruct->theUserStruct->dummy_variable = 0;
	theCtrlStruct->theUserStruct->t_last = theCtrlStruct->theCtrlIn->t;
	theCtrlStruct->theUserStruct->Ki1 = 0.0482;
	theCtrlStruct->theUserStruct->Kp1 = 0.3682;
	theCtrlStruct->theUserStruct->term2_r = 0.0;
	theCtrlStruct->theUserStruct->term2_l = 0.0;
	theCtrlStruct->theUserStruct->used_term2_r = 0.0;
	theCtrlStruct->theUserStruct->used_term2_l = 0.0;
	theCtrlStruct->theUserStruct->ratio; 

	return;
}

