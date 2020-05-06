#ifndef PATH_FOLLOW_HH
#define PATH_FOLLOW_HH

#include "namespace_ctrl.h"
#include "CtrlStruct_gr2.h"
#include "math.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "speed_regulation_gr2.h"
#include "user_realtime.h"

NAMESPACE_INIT(ctrlGr2);


int path_follow(CtrlStruct *cvs);

NAMESPACE_CLOSE();
#endif