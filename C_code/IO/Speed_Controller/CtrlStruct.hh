#ifndef CTRLSTRUCT_HH
#define CTRLSTRUCT_HH

#include "ctrl_io.h"

enum
{
    WAIT_STATE,
    CALIB_STATE,
    PINCHER_DEMO_STATE,
    AVOID150_STATE,
    RUN_STATE,
    STOP_STATE
};

//Behavior structure (Rudimentary)
enum
{
    FOLOW_STATE,
    AVOID_STATE
};

enum
{
    CALIB_1,
    CALIB_2,
    CALIB_3,
    CALIB_4
};

//Structure for the odometry
typedef struct RobotPosition
{
    double x;
    double y;
    double theta;
    double thetaref;

    double t_used;

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
} RobotParameters;

typedef struct MotStruct
{
    double kp; // Proportional param
    double ki; // Integral param
    double kd;
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
    int main_states;
    int calib_states;
    int pinchers_demo_states;
    int avoid150_states;
    double stopvalues[2];
    double t_ref;
} CtrlStruct;

int size_UserStruct();
void init_ctrlStruc(CtrlStruct *ctrl);

#endif