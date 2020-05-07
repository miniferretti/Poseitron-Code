#include "CtrlStruct.hh"

int size_UserStruct()
{
    return sizeof(CtrlStruct) * 10;
}

void init_ctrlStruc(CtrlStruct *ctrl)
{
    ctrl->theCtrlIn = new CtrlIn;
    ctrl->theUserStruct = new UserStruct;
    ctrl->theCtrlOut = new CtrlOut;
    ctrl->theUserStruct->theMotLeft = new MotStruct;
    ctrl->theUserStruct->theMotRight = new MotStruct;

	// path-planning
	ctrl->path = new PathPlanning;
	ctrl->path->xlen = 201; // 2m 
	ctrl->path->ylen = 201; // 2m
	ctrl->path->M = MatrixXd::Zero(ctrl->path->xlen, ctrl->path->ylen);
	ctrl->path->U = MatrixXd::Zero(ctrl->path->xlen, ctrl->path->ylen);
	ctrl->path->minObs = MatrixXd::Zero(ctrl->path->Obs.rows(), 1);
	ctrl->path->k_att = 10.0;
	ctrl->path->k_rep = 5.0;
	ctrl->path->rho_zero = 22.0;
	ctrl->path->path_changed = 0.0;
	ctrl->path->traj = MatrixXd::Zero(1, 2);
	ctrl->path->Opp = MatrixXd::Zero(ctrl->path->xlen, ctrl->path->ylen);
	ctrl->path->flag_repulsive = 1;
	ctrl->path->intermediary = 0;
	obstacle_building(ctrl->path);
    
	//Structure for the path-following algorithm
	ctrl->follower = new PathFollow;

	ctrl->follower->omega_sat = 1; 
	ctrl->follower->speed_sat = 0.5; 
	ctrl->follower->target = 0;
	ctrl->follower->count = 0;
	ctrl->follower->next = 1;
	ctrl->follower->alpha = 0;
	ctrl->follower->beta = 0;
	ctrl->follower->Krho = 1.5; //Krho > 0 otherwise unstable...
	ctrl->follower->Kalpha = 4; // Kalpha > Krho otherwise unstable...
	ctrl->follower->Kbeta = -0.75;
	ctrl->follower->last = 0;
	ctrl->follower->v_changed = 0;
	ctrl->follower->w_changed = 0;
    ctrl->follower->rhoLimit = 0.08; 
}
void obstacle_building(PathPlanning *path)
{
	int i, n, n_before, n_tot;
	// WALL CONSTRUCTION
	// bottom wall
	path->Obs.conservativeResize(path->xlen, 2);
	for (i = 0; i < path->xlen; i++)
	{
		path->Obs(i, 0) = (i - 100);
		path->Obs(i, 1) = -100;
	}
	n_before = path->xlen;

	// upper wall
	n = path->xlen;
	n_tot = n_before + n;
	path->Obs.conservativeResize(n_tot, 2);
	for (i = 0; i < n; i++)
	{
		path->Obs(i + n_before, 0) = (i - 100);
		path->Obs(i + n_before, 1) = 100;
	}
	n_before = n_tot;

	// left wall
	n = path->ylen;
	n_tot = n_before + n;
	path->Obs.conservativeResize(n_tot, 2);
	for (i = 0; i < n; i++)
	{
		path->Obs(i + n_before, 0) = -100;
		path->Obs(i + n_before, 1) = (i - 100);
	}
	n_before = n_tot;

	// right wall
	n = path->ylen;
	n_tot = n_before + n;
	path->Obs.conservativeResize(n_tot, 2);
	for (i = 0; i < n; i++)
	{
		path->Obs(i + n_before, 0) = 100;
		path->Obs(i + n_before, 1) = (i - 100);
	}
	n_before = n_tot;

}

void target_init(CtrlStruct *ctrl)
{
	Strategy *strat;
	strat = cvs->strat;

	// target 1
	strat->target(0, 0) = 0;			// coordonnée en x en m 
	strat->target(0, 1) = 0;			// coordonnée en y en m
	strat->target(0, 2) = 1;			// deja pris ou non 
	strat->target(0, 3) = 1;			// nombre de points
}