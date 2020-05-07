#include "ctrl_io.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

// State of the main
enum
{
    TEST_PATH_STATE,
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
    
    // Saturation of reference speed given to the speed controller 
    double speed_sat; 
    double omega_sat; 

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
	Eigen::MatrixXd M;
	Eigen::MatrixXd Obs;
	Eigen::MatrixXd Opp; 
	Eigen::MatrixXd minObs;
	Eigen::MatrixXd traj;
	Eigen::MatrixXd U;
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
    int number_of_succes;
    int number_of_pinch;
    int pinch_flag;
    FILE *RGBLog;

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
    double kp;             // Proportional param
    double ki;             // Integral param
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
} UserStruct;

typedef struct CtrlStruct
{
    UserStruct *theUserStruct; ///< user defined CtrlStruct
    CtrlIn *theCtrlIn;         ///< controller inputs
    CtrlOut *theCtrlOut;       ///< controller outputs
} CtrlStruct;

int size_UserStruct();
void init_ctrlStruc(CtrlStruct *ctrl);