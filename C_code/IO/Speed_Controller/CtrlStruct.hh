#include "ctrl_io.h"

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

int size_UserStruct();
void init_ctrlStruc(CtrlStruct *ctrl);