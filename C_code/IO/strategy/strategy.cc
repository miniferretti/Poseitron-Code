

#include "strategy.h"
#include "IO/path/path_planning.h"

void main_strategy(CtrlStruct *ctrl, P_Struct *my_P_Struct, SpeedController *spd)
{
	// variables declaration
	Strategy *strat;
	PathFollow *follower;
	CtrlIn *inputs;
	PathPlanning *path;

	void **retval;

	int follow;
	int redzone;
	int goalx, goaly, x, y;
	double TIME_FUN1 = 0.1; // in sec

	// variables initialization
	inputs = ctrl->theCtrlIn;
	strat = ctrl->strat;
	path = ctrl->path;
	follower = ctrl->follower;
	switch (strat->state)
	{
	case STRAT_STATE_PATH:

		// Thread creation
		if (my_P_Struct->p_path_update_flag == 0)
		{
			printf("\nNew path\n\r");
			my_P_Struct->p_path_update_flag = !pthread_create(&my_P_Struct->p_path_update, NULL, &path_planning_update, (void *)ctrl);
		}
		// Check that the thread has completed its computation

		else if (pthread_tryjoin_np(my_P_Struct->p_path_update, retval) == 0)
		{
			my_P_Struct->p_path_update_flag = 0;
			strat->state = STRAT_STATE_FOLLOW;
		}
		break;

	case STRAT_STATE_OPPONENT_AVOIDANCE:
		// Thread creation
		if (my_P_Struct->p_avoidance_path_flag == 0)
		{
			printf("/////////////////////////////////////\n\r");
			printf("\nOpponent avoidance : new path\n\n");
			my_P_Struct->p_avoidance_path_flag = !pthread_create(&my_P_Struct->p_avoidance_path, NULL, &avoidance_path_update, (void *)ctrl);
		}
		// Check that the thread has completed its computation
		else if (pthread_tryjoin_np(my_P_Struct->p_avoidance_path, retval) == 0)
		{
			my_P_Struct->p_avoidance_path_flag = 0;
			ctrl->opp_pos->flag = 0;
			strat->state = STRAT_STATE_FOLLOW;
			printf("/////////////////////////////////////\n\r");
		}
		break;

	case STRAT_STATE_FOLLOW:
		/* Middle controller */
		follow = path_follow(ctrl);
		redzone = opponent_detection(ctrl);
		// Needed if someone says that the path needs to be updated
		if (ctrl->opp_pos->flag == 1)//redzone)
		{
			printf(">>>	new path for avoidance\n");
			spd->set_speed(0.0, 0.0);
			strat->state = STRAT_STATE_OPPONENT_AVOIDANCE;
		}
		/* End of path and target position achived */
		if (follow)
		{
			strat->state = STRAT_STATE_PATH_END;
		}
		break;

	case STRAT_STATE_PATH_END:
		printf("\nStrat path end \n\r");
		printf("/////////////////////////////////////\n\n\r");

		if (path->intermediary == 1)
		{
			printf(">>>	intermediary point achieved\n");
			path->intermediary = 0;
			strat->state = STRAT_STATE_PATH;
		}

		else if (follower->flag_rho == 1)
		{
			// In case where the robot is not close enough from the target.
			// The limit is lowered in order to resume again the finished path.
			printf(">>>	limit lowered to %f\n", 0.03);
			follower->count = path->traj.rows() - 5 - 1;
			follower->next = 1;
			follower->rhoLimit = 0.03;
			follower->flag_rho = 0;
			strat->state = STRAT_STATE_FOLLOW;
		}
		else
		{
			// detection de qque chose
			strat->tref = inputs->t;
			follower->flag_rho = 1;
			follower->rhoLimit = 0.05; // Limit from the target reset.
			strat->state = STRAT_STATE_WAIT;
		}
		break;

	case STRAT_STATE_WAIT:
		spd->set_speed(0.0, 0.0);
		if (inputs->t - strat->tref > TIME_FUN1)
		{
			/// HERE AN OTHER FUNCTION IS PUT.

			strat->target((int)follower->target, 3) = 0;
			printf("\n\r>>>	function finished \n");
			strat->wait_count = 0;

			if (false)//follower->target == 3)
			{
				ctrl->main_states = STOP_STATE;
				fclose(ctrl->follower->logFile);

				strat->state = STRAT_STATE_PATH;
				ctrl->flag_state = 0;
			}
			else
			{
				strat->state = STRAT_STATE_GOAL;
			}
		}
		if (strat->wait_count == 0)
		{
			printf("\n\r>>>	waiting for function to be finished ...\n\r");
			strat->wait_count = 1;
		}

		break;

	case STRAT_STATE_GOAL:
		printf("\n////////////////////////////////////////\n\r");
		printf("\n\r>>>	search for new goal ...\n\r");
		if (follower->target == 4){
			follower->target = 0; 
		}
		else {
			follower->target++;
		}
		
		if (follower->target == 1){
			ctrl->opp_pos->flag = 1;
		}
		
		// Next state
		if (false)
		{
			path->intermediary = 1;
			strat->state = STRAT_STATE_PATH;
		}
		else
		{
			strat->state = STRAT_STATE_PATH;
		}
		break;

	default:
		printf("Strategy error: unknown state: %d !\n", strat->state);
		exit(EXIT_FAILURE);
	}
}

int opponent_detection(CtrlStruct *ctrl)
{
	return 0;
}