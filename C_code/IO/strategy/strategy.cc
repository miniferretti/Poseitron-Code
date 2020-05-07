



#include "strategy.h"
#include "IO/path/path_planning.h"
#include "IO/Speed_Controller/SpeedController.hh"


NAMESPACE_INIT(ctrlGr2);

/*! \brief startegy during the game
 * 
 * \param[in,out] ctrl controller main structure
 */
void main_strategy(CtrlStruct *ctrl)
{
	// variables declaration
	Strategy *strat;
	PathFollow *follower;
	CtrlIn *inputs; 
	PathPlanning *path; 
	int follow ; 
	int redzone; 
	int goalx, goaly, x , y;
	double TIME_FUN1 = 2.0; // in sec

	// variables initialization
	inputs = ctrl->theCtrlIn; 
	strat = ctrl->strat;
	path = ctrl->path; 
	follower = ctrl->follower;
	switch (strat->state)
	{
		case STRAT_STATE_PATH:
		printf("\nNew path\n\r");
		goalx = (int) (ctrl->strat->target((int) ctrl->follower->target,0)*100);
		goaly = (int) (ctrl->strat->target((int) ctrl->follower->target,1)*100);
		path_planning_update(ctrl, goalx, goaly); // update the path-planning
		strat->state = STRAT_STATE_FOLLOW;
		break;

		case STRAT_STATE_INTERMEDIARY_PATH:
		printf("\n\rIntermediary path\n\r");
		goalx = 45; 
		goaly = 0; 
		path_planning_update(ctrl, goalx, goaly);
		strat->state = STRAT_STATE_FOLLOW;
		break;

		case STRAT_STATE_OPPONENT_AVOIDANCE:
        printf("/////////////////////////////////////\n\r");
		printf("\nOpponent avoidance : new path\n\n");
		avoidance_path_update(ctrl);
        printf("/////////////////////////////////////\n\r");
		strat->state = STRAT_STATE_FOLLOW;
		break; 

		case STRAT_STATE_FOLLOW:
		/* Middle controller */
		follow = path_follow(ctrl);
		redzone = opponentDetection(ctrl);
		// Needed if someone says that the path needs to be updated 
		if (redzone){
			printf(">>>	new path for avoidance\n");
			strat->state = STRAT_STATE_OPPONENT_AVOIDANCE;
		}
		/* End of path and target position achived */
		if (follow){
			strat->state = STRAT_STATE_PATH_END;
		}
		break;

		case STRAT_STATE_PATH_END:
        printf("\nStrat path end \n\r");
        printf("/////////////////////////////////////\n\n\r");

		if (path->intermediary == 1){
			printf(">>>	intermediary point achieved\n");
			path->intermediary = 0;

			strat->state = STRAT_STATE_PATH;
		}
		else if (true){
			// detection de qque chose
			strat->tref = inputs->t; 

			strat->state = STRAT_STATE_WAIT;
		}
		else{
			//printf(">>>	target %d not detected\n", (int) follower->target);
   		    printf(">>>	limit lowered to %f\n",follower->rhoLimit - 0.01);
   		    follower->count = path->traj.rows() - 5 - 1;
			follower->next = 1;
			follower->rhoLimit -= 0.01; 

			strat->state = STRAT_STATE_FOLLOW;
		}
		break; 

		case STRAT_STATE_WAIT:
		set_speed(0.0, 0.0);
		follower->rhoLimit = 0.08; // on remet la valeur de base 
		if (inputs->t - strat->tref > TIME_FUN1){
			if (strat->wait_count == 0){
				printf("\n\r>>>	waiting for function to be finished ...\n\r");
				strat->wait_count = 1;
			}
			strat->target((int)follower->target,3) = 0;
			printf("\n\r>>>	function finished \n");
			strat->wait_count = 0; 
			ctrl->main_states = STOP_STATE;
		}
		break;

		case STRAT_STATE_GOAL:
		printf("\n////////////////////////////////////////\n\r");
		printf("\n\r>>>	search for new goal ...\n\r");
		
		// Next state 
		if (false){
			strat->state = STRAT_STATE_INTERMEDIARY_PATH;
		}
		else{
			strat->state = STRAT_STATE_PATH;
		}
		break;


		default:
		printf("Strategy error: unknown state: %d !\n", strat->state);
		exit(EXIT_FAILURE);
	}
}

int opponentDetection(CtrlStruct *ctrl){
	return 0; 
}
NAMESPACE_CLOSE();
