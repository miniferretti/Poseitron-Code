/*! 
 * \author GR2
 * \file path_planning_ex.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_H_
#define _PATH_PLANNING_H_

#include "IO/Speed_Controller/CtrlStruct.hh"
#include <math.h>

// The map without the opponent is kept in memory in order to
// not multiply the same computation twice.
// So at each t*, just the potential field regarding the position
// of the opponent is updated (repulsive_opp_potential_field)
// Only if the goal changes, the attractive potential field
// will update.
void *path_planning_update(void *myCtrl); // attention qu'un seul argument de type void !!!
void *avoidance_path_update(void *myCtrl);

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

#endif
