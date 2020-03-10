#include "ctrl_io.h"

typedef struct UserStruct
{
    double Ra;
    double kphi;

    double kpr;
    double kir;

    double kpl;
    double kil;

    double t_p;
    double ratio;
    double upperCurrentLimit;
    double lowerCurrentLimit;
    double upperVoltageLimit;
    double lowerVoltageLimit;

    double tics;
    double samplingDE0;

    double i_e_l;
    double i_e_r;

    int sat_l;
    int sat_r;
} UserStruct;

typedef struct CtrlStruct
{
    UserStruct *theUserStruct; ///< user defined CtrlStruct
    CtrlIn *theCtrlIn;         ///< controller inputs
    CtrlOut *theCtrlOut;       ///< controller outputs
} CtrlStruct;

int size_UserStruct();
//void init_ctrlStruc(CtrlStruct *ctrl);