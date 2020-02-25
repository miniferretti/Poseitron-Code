#include "IO/ctrl_io.h"



typedef struct UserStruct{
	double Ra;
	double kphi;
	double Kp;
	double Ki;
    double t_p;
    double ratio; 
    double upperCurrentLimit;
    double lowerCurrentLimit;
    double upperVoltageLimit;
    double lowerVoltageLimit;

    double i_e_l;
    double i_e_r;

    int sat_l;
    int sat_r;
} UserStruct;


typedef struct CtrlStruct
{
	UserStruct *theUserStruct; ///< user defined CtrlStruct
	CtrlIn *theCtrlIn;		   ///< controller inputs
	CtrlOut *theCtrlOut;	   ///< controller outputs
} CtrlStruct;

typedef struct CtrlIn{
    double t; 
    double wheel_speed_r;
    double wheel_speed_l;

} CtrlIn;

typedef struct CtrlOut{
    double wheel_commands_l;
    double wheel_commands_r;
} CtrlOut;
int size_UserStruct();