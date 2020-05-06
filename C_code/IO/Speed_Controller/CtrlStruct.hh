#ifndef CTRLSTRUCT_HH
#define CTRLSTRUCT_HH

#include "ctrl_io.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>


enum
{
    DUMMY_STATE,
    WAIT_STATE,
    ODO_CALIB_STATE,
    CALIB_STATE,
    PINCHER_DEMO_STATE,
    AVOID150_STATE,
    RUN_STATE,
    SlAVE_STATE,
    STOP_STATE
};

//Behavior structure (Rudimentary)
enum
{
    FOLOW_STATE,
    AVOID_STATE
};

// strategy main states
enum
{
	STRAT_STATE_PATH,
	STRAT_STATE_INTERMEDIARY_PATH,
	STRAT_STATE_PATH_END,
	STRAT_STATE_OPPONENT_AVOIDANCE,
	STRAT_STATE_FOLLOW, // look for opponent all the time
	STRAT_STATE_WAIT, 
	STRAT_STATE_GOAL // target or base
};
/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time to save

	int flag; ///< flag for calibration

	int loop;

	double calib_speed;

	double basis_center_x;

	int count; 
} RobotCalibration;

// path follow structure
typedef struct PathFollow
{	
	int count;
	int next;
	int last;
	double target;

	double rho;
	double alpha;
	double beta;

	//Coeficients of the controll loop 
	double Krho;
	double Kalpha;
	double Kbeta;

	double v_changed;
	double w_changed;

	double rhoLimit;

} PathFollow;


// path-planning main structure
typedef struct PathPlanning
{
	int xlen;
	int ylen;
	MatrixXd M;
	MatrixXd Obs;
	MatrixXd Opp; 
	MatrixXd minObs;
	MatrixXd traj;
	MatrixXd U;
	int Obslen;
	double k_att;
	double k_rep;
	double rho_zero;
	int path_changed;
	int flag_repulsive; 
	int intermediary; 

} PathPlanning;

/// strategy
typedef struct Strategy
{
	int state; ///< main state of the strategy
	int count; 
	MatrixXd target; 
	int tref; 
	int wait_count; 
} Strategy;

//Structure for the odometry
typedef struct RobotPosition
{
    double x;
    double x_prev;

    double y;
    double y_prev;

    double x_ref;
    double y_ref;

    double theta;

    double dtheta;
    double thetaref;
    int dtheta_flag;

    double dist;
    double dist_prev;

    double ddist;
    double ddist_flag;

    double t_used;

    double kl;
    double kr;

    double dydx;
    double dydx_prev;

    int ignore;


    //Compensation factors for correcting the non idealities of the robot anatomy



    Eigen::MatrixXd Dpf;
    Eigen::MatrixXd Drlf;
    Eigen::MatrixXd covs;
    Eigen::MatrixXd error;

} RobotPosition;

typedef struct RobotPinchers
{
    bool stateL[3]; //true = open, false = closed
    bool stateR[3]; //true = open, false = closed

} RobotPinchers;

typedef struct RobotParameters
{
    double odo_radius;
    double odo_tics_per_rot;
    double robot_width; //Length between the two odometers
    
	double wheel_rad;
	double wheel_dist;
	double center_to_back_dist;
} RobotParameters;

typedef struct MotStruct
{
    double kp; // Proportional param
    double ki; // Integral param
    double kd;
    int ki_flag;
    double integral_error; // last integral error
    double status;         // Is or not in saturation

    // General motor parameter
    double Ra;
    double kphi;
    double t_p;
    double ratio;
    double upperCurrentLimit;
    double lowerCurrentLimit;
    double upperVoltageLimit;
    double lowerVoltageLimit;


    double compensation_factor; 
} MotStruct;
typedef struct UserStruct
{
    // Structure of motors
    MotStruct *theMotLeft;
    MotStruct *theMotRight;


    // additional param
    double tics;
    double samplingDE0;
    int speed_kill;
    int Odo_kill;
} UserStruct;

typedef struct CtrlStruct
{
    UserStruct *theUserStruct; ///< user defined CtrlStruct
    CtrlIn *theCtrlIn;         ///< controller inputs
    CtrlOut *theCtrlOut;       ///< controller outputs
    RobotPosition *rob_pos;
    RobotParameters *robot;
    RobotPinchers *pinchers;


    //////////// NEW ADDED ////////////////////////////
	RobotCalibration *calib;	  ///< calibration
	PathPlanning *path;			  ///< path-planning
	PathFollow *follower;         ///< Follow path given by path planning
	Strategy *strat;			  ///< strategy
    ///////////////////////////////////////////////////


    int main_states;
    int calib_states;
    int pinchers_demo_states;
    int avoid150_states;
    int odo_calibration_states;
    double stopvalues[2];
    double t_ref;
    double main_t_ref;
} CtrlStruct;

int size_UserStruct();
void init_ctrlStruc(CtrlStruct *ctrl);

/////////// NEW ADDED /////////////
void obstacle_building(PathPlanning *path);
void target_init(CtrlStruct *ctrl);
///////////////////////////////////

#endif