



#include "strategy.h"
#include "path_planning.h"
#include "speed_regulation_gr2.h"

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
	CtrlOut *outputs; 
	PathPlanning *path; 
	int follow ; 
	int redzoneGlobal = 0, redzone1 = 0, redzone2 = 0, redzone3 = 0;
	double dx1, dx2, dx3,dy1, dy2, dy3, dist1, dist_opp;
	int goalx, goaly, x , y;

	// variables initialization
	inputs = ctrl->inputs; 
	strat = ctrl->strat;
	path = ctrl->path; 
	follower = ctrl->follower;
	outputs = ctrl->outputs;
	switch (strat->state)
	{
		case STRAT_STATE_PATH:
		printf("\nStrat path planning\n\r");
		path_planning_update(ctrl); // update the path-planning

		strat->state = STRAT_STATE_FOLLOW;
		break;

		case STRAT_STATE_INTERMEDIARY_PATH:
		printf("\n\r>>>	Intermediary path\n\r");
		goalx = 45; goaly = 0; 
		x = (int) (ctrl->rob_pos->x*100); y = (int) (ctrl->rob_pos->y*100);
		printf(">>>	path : initial position (%d, %d) & goal position (%d, %d) \n\r",x, y, goalx, goaly);
		attractive_potential_field(ctrl, goalx, goaly);
		solve_path(ctrl,x, y, goalx, goaly);
		printf(">>>	path computation done\n");

		strat->state = STRAT_STATE_FOLLOW;
		break;

		case STRAT_STATE_OPPONENT_AVOIDANCE:
        printf("/////////////////////////////////////\n\r");
		printf("\nStrat opponent avoidance\n");
		goalx = (int) (strat->target((int)follower->target,0)*100); 
		goaly = (int) (strat->target((int)follower->target,1)*100);
		// Path solving : Keeping same target
		repulsive_opp_potential_field(ctrl);
		solve_path(ctrl, (int) (ctrl->rob_pos->x*100), (int) (ctrl->rob_pos->y*100), goalx, goaly);
		printf(">>>	path computation done\n");

		strat->state = STRAT_STATE_FOLLOW;
		break; 

		case STRAT_STATE_FOLLOW:
			//printf("Strat follow \n\r");
			//printf("detected %d \n \r", ctrl->inputs->target_detected);
			//printf("nb_targets %d \n \r", ctrl->inputs->nb_targets);
			//printf("row = %d \n\r", (int) ctrl->path->traj.rows());

			/* Opponent position decision for path planning update */
			/*
			dist_opp = 0; 

			// opponent 1
			dx1 = ctrl->rob_pos->x - ctrl->opp_pos->x1;
			dy1 = ctrl->rob_pos->y - ctrl->opp_pos->y1;
			dist1 = sqrt(dx1*dx1 + dy1*dy1); 
			redzone1 = (dist1 < dist_opp);
			printf("dist = %f\n ", ctrl->opp_pos->d);

			// opponent 2
			dx2 = path->traj(follower->count, 0) / 100 - ctrl->opp_pos->x2;
			dy2 = path->traj(follower->count, 1) / 100 - ctrl->opp_pos->y2;
			redzone2 = (sqrt(dx2*dx2 + dy2*dy2) < dist_opp);

			// opponent 3
			dx3 = path->traj(follower->count, 0) / 100 - ctrl->opp_pos->x3;
			dy3 = path->traj(follower->count, 1) / 100 - ctrl->opp_pos->y3;
			redzone3 = (sqrt(dx3*dx3 + dy3*dy3) < dist_opp);

			redzoneGlobal = redzone1 || redzone2 || redzone3;
			*/	
		outputs->flag_release = 0; 
		
		/* Middle controller */
		follow = path_follow(ctrl);

		if (strat->count == 0){
			strat->count = 1;
			printf(">>>	new path for avoidance\n");
			strat->state = STRAT_STATE_OPPONENT_AVOIDANCE;
		}

		/* End of path and target position achived */
		if (follow == 1){
			strat->state = STRAT_STATE_PATH_END;
		}
		break;

		case STRAT_STATE_PATH_END:
        printf("\nStrat path end \n\r");
        printf("/////////////////////////////////////\n\n\r");

		if (path->intermediary == 1){
			//set_speed(ctrl, 0.0, 0.0);
			printf(">>>	intermediary point achieved\n");
			set_speed(ctrl, 0.0, 0.0);
			path->intermediary = 0;

			strat->state = STRAT_STATE_PATH;
		}
		else if ((follower->target != 7 && inputs->target_detected) || follower->target == 7){
			printf(">>>	target %d detected\n", (int) follower->target);
			strat->tref = inputs->t; 

			strat->state = STRAT_STATE_WAIT;
		}
		else{
			printf(">>>	target %d not detected\n", (int) follower->target);
   		    printf(">>>	limit lowered to %f\n",follower->rhoLimit - 0.01);
   		    follower->count = path->traj.rows() - 5 - 1;
			follower->next = 1;
			follower->rhoLimit -= 0.01; 

			strat->state = STRAT_STATE_FOLLOW;
		}
		break; 

		case STRAT_STATE_WAIT:
		set_speed(ctrl, 0.0, 0.0);
		follower->rhoLimit = 0.08; // on remet la valeur de base 
		if (follower->target == 7){
			if (strat->wait_count == 0){
				printf("\n\r>>>	target released...\n\r");
				strat->wait_count = 1;
			}
			outputs->flag_release = 1; 
			strat->wait_count = 0; 

			strat->state = STRAT_STATE_GOAL;
		}
		else if (inputs->t - strat->tref > 2.5){
			if (strat->wait_count == 0){
				printf("\n\r>>>	waiting for target to be picked up ...\n\r");
				strat->wait_count = 1;
			}
			strat->target((int)follower->target,3) = 0;
			printf("\n\r>>>	target %d taken\n", (int) follower->target);
			//printf("number of target detected %d\n", (int) inputs->nb_targets);
			strat->wait_count = 0; 

			strat->state = STRAT_STATE_GOAL;
		}
		break;

		case STRAT_STATE_GOAL:
		printf("\n////////////////////////////////////////\n\r");
		printf("\n\r>>>	search for new goal ...\n\r");
		if (inputs->nb_targets == 2){
			printf(">>>	target 7 (basis) chosen (value 0)\n\r");
			follower->target = 7;
		}
		else{
			for (int i = 6; i >= 0 ; i--){
				//printf("target %d taken? = %f \n\r", i, strat->target(i,3));
				if (strat->target(i,3) == 1){
					printf(">>>	target %d chosen (value %d)\n\r", i, (int) strat->target(i,2));
					follower->target = i;
					break;
				}
			}
		}
		if (follower->target == 7 || follower->target == 4){
			path->intermediary = 1; 
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

NAMESPACE_CLOSE();
