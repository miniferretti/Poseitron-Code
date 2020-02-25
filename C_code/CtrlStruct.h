#include "IO/ctrl_io.h"



typedef struct UserStruct
{
	//Robot parameters :
	int ratio; 
	//geometric and sensors
	double positions_per_rev_beacon;
	double positions_per_rev_roues;
	double direction_ref;
	double distance_ref;


	//Beacon parameters
	double diameter;
	double beacon_angle;
	double beacon_direction;
	double beacon_distance;
	double beacon_detect;

	//wheel speed controller
	double t_last;
	double Ki1;
	double Kp1;
	double term2_l;
	double term2_r;
	double used_term2_l;
	double used_term2_r;

	//beacon distance controller
	double t_last_dist;
	double Ki2;
	double Kp2;
	double term2_dist;

	//beacon direction controller

	//wheel speed references
	double *omega_ref;

} UserStruct;

typedef struct CtrlStruct
{
	UserStruct *theUserStruct; ///< user defined CtrlStruct
	CtrlIn *theCtrlIn;		   ///< controller inputs
	CtrlOut *theCtrlOut;	   ///< controller outputs
} CtrlStruct;

int size_UserStruct();
