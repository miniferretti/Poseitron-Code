

#include "CtrlStruct.hh"
#include "IO/Mid_level_controller/Avoid150.hh"
#include "IO/Pincher_Demo/Pincher_Demo.hh"
#include "IO/Calibration/Calibration.hh"
#include "IO/Calibration/odo_calibration.h"
//#include "IO/strategy/strategy.h"
//#include "IO/path/path_planning.h"
//#include "IO/Mid_level_controller/path_follow.h"

int size_UserStruct()
{
	return sizeof(CtrlStruct) * 10;
}

void init_P_Struct(P_Struct *my_P_Struct)
{
	my_P_Struct->p_avoidance_path_flag = 0;
	my_P_Struct->p_path_update_flag = 0;
}

void init_ctrlStruc(CtrlStruct *ctrl)
{
	ctrl->theCtrlIn = new CtrlIn;
	ctrl->theUserStruct = new UserStruct;
	ctrl->theCtrlOut = new CtrlOut;
	ctrl->theUserStruct->theMotLeft = new MotStruct;
	ctrl->theUserStruct->theMotRight = new MotStruct;
	ctrl->rob_pos = new RobotPosition;
	ctrl->robot = new RobotParameters;
	ctrl->robot->wheel_rad = 0.03;
	ctrl->robot->wheel_dist = 0.19; // 19cm
	ctrl->pinchers = new RobotPinchers;
	ctrl->pinchers->number_of_succes = 0;
	ctrl->pinchers->number_of_pinch = 0;
	ctrl->pinchers->pinch_flag = 0;
	ctrl->pinchers->RGBLog = fopen("/home/pi/Poseitron-Code/Data/RGBLog.txt", "w");
	ctrl->avoid150_states = AVOID150_STATE1;
	ctrl->pinchers_demo_states = SETUP_STATE;
	ctrl->calib_states = CALIB_1;
	ctrl->odo_calibration_states = GO_STRAIGHT;
	ctrl->main_t_ref = 0;
	ctrl->flag_state = 1; 

	// calibration
	ctrl->calib = new RobotCalibration;

	ctrl->calib->flag = 0;
	ctrl->calib->t_flag = 0.0;
	ctrl->calib->loop = 0;
	ctrl->calib->calib_speed = 10;
	ctrl->calib->basis_center_x = -0.2;
	ctrl->calib->count = 0;

	// strategy
	ctrl->strat = new Strategy;
	ctrl->strat->state = STRAT_STATE_PATH;
	ctrl->strat->target = Eigen::MatrixXd::Zero(8, 5);
	ctrl->strat->tref = 0;
	ctrl->strat->count = 0;
	ctrl->strat->wait_count = 0;
	target_init(ctrl);

	// path-planning
	ctrl->path = new PathPlanning;
	ctrl->path->xlen = 201;
	ctrl->path->ylen = 201;
	ctrl->path->M = Eigen::MatrixXd::Zero(ctrl->path->xlen, ctrl->path->ylen);
	ctrl->path->U = Eigen::MatrixXd::Zero(ctrl->path->xlen, ctrl->path->ylen);
	obstacle_building(ctrl->path);
	ctrl->path->minObs = Eigen::MatrixXd::Zero(ctrl->path->Obs.rows(), 1);
	ctrl->path->k_att = 10.0;
	ctrl->path->k_rep = 5.0;
	ctrl->path->rho_zero = 22.0;
	ctrl->path->path_changed = 0.0;
	ctrl->path->traj = Eigen::MatrixXd::Zero(1, 2);
	ctrl->path->Opp = Eigen::MatrixXd::Zero(ctrl->path->xlen, ctrl->path->ylen);
	ctrl->path->flag_repulsive = 1;
	ctrl->path->intermediary = 0;

	//Structure for the path-following algorithm
	ctrl->follower = new PathFollow;
	ctrl->follower->omega_sat = 3*1.5;
	ctrl->follower->speed_sat = 1.8;
	ctrl->follower->target = 0;
	ctrl->follower->count = 0;
	ctrl->follower->next = 1;
	ctrl->follower->alpha = 0;
	ctrl->follower->beta = 0;
	ctrl->follower->prop_param = 4.4;
	ctrl->follower->Krho = 1.5*2*2*1.1; //Krho > 0 otherwise unstable...
	ctrl->follower->Kalpha = 4*2*2*1.1; // Kalpha > Krho otherwise unstable...
	ctrl->follower->Kbeta = -0.75*2*2*1.1;
	ctrl->follower->last = 0;
	ctrl->follower->v_changed = 0;
	ctrl->follower->w_changed = 0;
	ctrl->follower->rhoLimit = 0.05;
	ctrl->follower->flag_rho = 1;
	ctrl->follower->logFile = fopen("/home/pi/Poseitron-Code/Data/FollowerFile.txt", "w"); 
}

void obstacle_building(PathPlanning *path)
{
	int i, n, n_before, n_tot;
	n_before = 0; 
	n = path->xlen;
	n_tot = n_before + n;
	// WALL CONSTRUCTION
	// bottom wall
	path->Obs.conservativeResize(n_tot, 2);
	for (i = 0; i < n; i++)
	{
		path->Obs(i + n_before, 0) = (i - 100);
		path->Obs(i + n_before, 1) = -100;
	}
	n_before = path->xlen;

	// upper wall
	n = path->xlen;
	n_tot = n_before + n;
	path->Obs.conservativeResize(n_tot, 2);
	for (i = 0; i < n; i++)
	{
		path->Obs(i + n_before, 0) = (i - 100);
		path->Obs(i + n_before, 1) = 100;
	}
	n_before = n_tot;

	// left wall
	n = path->ylen;
	n_tot = n_before + n;
	path->Obs.conservativeResize(n_tot, 2);
	for (i = 0; i < n; i++)
	{
		path->Obs(i + n_before, 0) = -100;
		path->Obs(i + n_before, 1) = (i - 100);
	}
	n_before = n_tot;

	// right wall
	n = path->ylen;
	n_tot = n_before + n;
	path->Obs.conservativeResize(n_tot, 2);
	for (i = 0; i < n; i++)
	{
		path->Obs(i + n_before, 0) = 100;
		path->Obs(i + n_before, 1) = (i - 100);
	}
	n_before = n_tot;
	printf("obs length = %d \n\r", path->Obs.rows());
}

void target_init(CtrlStruct *ctrl)
{
	Strategy *strat;
	strat = ctrl->strat;

	// target 1
	strat->target(0, 0) = 0.5; // coordonnée en x en m
	strat->target(0, 1) = 0.5;   // coordonnée en y en m
	strat->target(0, 2) = 1;   // deja pris ou non
	strat->target(0, 3) = 1;   // nombre de points

	// target 0
	strat->target(1, 0) = 0.5; // coordonnée en x en m
	strat->target(1, 1) = -0.5;   // coordonnée en y en m
	strat->target(1, 2) = 1;   // deja pris ou non
	strat->target(1, 3) = 1;   // nombre de points
	

	strat->target(2, 0) = -0.5; // coordonnée en x en m
	strat->target(2, 1) = 0.5;   // coordonnée en y en m
	strat->target(2, 2) = 1;   // deja pris ou non
	strat->target(2, 3) = 1;   // nombre de points

	strat->target(3, 0) = -0.5; // coordonnée en x en m
	strat->target(3, 1) = -0.5;   // coordonnée en y en m
	strat->target(3, 2) = 1;   // deja pris ou non
	strat->target(3, 3) = 1;   // nombre de points
}
/*
void repulsive_potential_field(CtrlStruct *ctrl){
	PathPlanning *path; 
	path = ctrl->path; 

	FILE *fp; 
	double value; 
	fp = fopen("RecordField.txt", "r"); 
	if (fp == NULL){
		printf("Error opening the file \n");
	}
	for (int j = 0; j<=path->ylen; j++ ){
		for (int i = 0; i<=path->xlen; i++){
			fscanf(fp, "%f\t", &value);
			path->M(i,j) = value; 
		}
		fscanf(fp, "%\n");
	}
	fclose(fp);
}
*/
