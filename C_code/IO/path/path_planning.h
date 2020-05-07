/*! 
 * \author GR2
 * \file path_planning_ex.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR2_H_
#define _PATH_PLANNING_GR2_H_

#include "CtrlStruct.h"
#include "namespace_ctrl.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "set_output.h"

NAMESPACE_INIT(ctrlGr2);
// The map without the opponent is kept in memory in order to 
// not multiply the same computation twice. 
// So at each t*, just the potential field regarding the position 
// of the opponent is updated (repulsive_opp_potential_field)
// Only if the goal changes, the attractive potential field
// will update.
void path_planning_update(CtrlStruct *ctrl); 

void attractive_potential_field(CtrlStruct *ctrl, int goalx, int goaly);
void attractive_potential_field_reverse(CtrlStruct *ctrl);
void repulsive_map_potential_field(PathPlanning *path);
void repulsive_opp_potential_field(CtrlStruct *ctrl);
void repulsive_opp_potential_field_reverse(CtrlStruct *ctrl);

// called each time we want to put an new obstacle in the potential field
// for ex: 	-At start to translate the map into a repulsive potential field 
// 			-During the game to take the opponent into account. 
//
void set_obstacle(PathPlanning *path);

// Knowing the overall potential field, this function is able to retreive 
// The path in order to reach the target. 
// If the region occupied by the opponent (all its covered radius) 
// in t* cut the path then this function is called to give a new path for the 
// robot to take.

void solve_path(CtrlStruct *ctrl, int x, int y, int goalx, int goaly); 

// Additional useful function
int isinbacktrack(PathPlanning *path, int x, int y); 
double distanceObs(PathPlanning *path, int x, int y);

NAMESPACE_CLOSE();

#endif