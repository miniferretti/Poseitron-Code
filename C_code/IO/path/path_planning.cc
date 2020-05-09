#include "IO/path/path_planning.h"

void *path_planning_update(void *myCtrl)
{
	CtrlStruct *ctrl = (CtrlStruct *)myCtrl;
	//printf("Path planning update\n\r");
	// Declaration of variable
	PathPlanning *path;
	path = ctrl->path;
	int x, y, goalx, goaly;
	if (path->intermediary == 1)
	{
		goalx = 45;
		goaly = 0;
	}
	else
	{
		goalx = (int)(ctrl->strat->target((int)ctrl->follower->target, 0) * 100);
		goaly = (int)(ctrl->strat->target((int)ctrl->follower->target, 1) * 100);
	}

	x = (int)(ctrl->rob_pos->x * 100);
	y = (int)(ctrl->rob_pos->y * 100);
	printf(">>> path : initial position (%d, %d) & goal position (%d, %d) \n\r", x, y, goalx, goaly);

	if (path->flag_repulsive == 1)
	{
		//repulsive_map_potential_field(path);
		//printf(">>> repulsive map computation done\n\r");
		ctrl->path->flag_repulsive = 0;
	}
	attractive_potential_field(ctrl, goalx, goaly);
	printf(">>> attractive potential field computation done\n\r");
	solve_path(ctrl, x, y, goalx, goaly);
	printf(">>> path computation done \n\r");

	printf("Show path \n\r");
	int n = (int)path->traj.rows();
	for (int i = 0; i < n; i++)
	{
		printf("%f %f\n\r", path->traj(i, 0), path->traj(i, 1));
	}

	/*	
	printf("Show Field \n\r");
	for (int i = 0; i < path->xlen ; i++){
		for (int j = 0; j < path->ylen ; j++){
			printf("%f ",path->M(i,j));
		}
		printf("\n\r");
	}
*/

	return NULL;
}

void *avoidance_path_update(void *myCtrl)
{
	CtrlStruct *ctrl = (CtrlStruct *)myCtrl;
	int goalx, goaly;
	goalx = (int)(ctrl->strat->target((int)ctrl->follower->target, 0) * 100);
	goaly = (int)(ctrl->strat->target((int)ctrl->follower->target, 1) * 100);

	//	repulsive_opp_potential_field(ctrl);
	solve_path(ctrl, (int)(ctrl->rob_pos->x * 100), (int)(ctrl->rob_pos->y * 100), goalx, goaly);
	printf(">>> path computation done\n");
	return NULL;
}

void repulsive_map_potential_field(PathPlanning *path)
{
	//path->RecordField = fopen("/home/pi/Poseitron-Code/Data/RecordField.txt", "w");
	set_obstacle(path);
	path->M = path->M.eval() / path->M.maxCoeff(); //normalize
	/*
	for (int i = 0; path->M.rows() > i; i++)
	{
		for (int j = 0; path->M.cols() > j; j++)
		{
			fprintf(path->RecordField, "%f\r\n", path->M(i, j));
		}
	}

	fclose(path->RecordField);
	*/
}

void set_obstacle(PathPlanning *path)
{
	int yval, xval;
	double dobs, U_rep;
	int countx = 0;
	int county = 0;
	int xinit = 100;
	int yinit = 100;
	//printf("Set obstacle \n\r");
	for (xval = -xinit; xval < path->xlen - xinit; xval++)
	{
		//printf("countx = %d\n\r", countx++);
		for (yval = -yinit; yval < path->ylen - yinit; yval++)
		{
			//printf("yval = %d county = %d\n\r", yval, county++);
			dobs = distanceObs(path, xval, yval);
			if (dobs == 0.0)
				U_rep = path->k_rep;
			else if (dobs < path->rho_zero)
			{
				U_rep = path->M(xval + xinit, yval + yinit) + path->k_rep * exp(-5 * dobs / path->rho_zero);
			}
			else
				U_rep = path->M(xval + xinit, yval + yinit);

			//printf("Urep[%d][%d] = %f \r\n",xval, yval, U_rep);

			path->M(xval + xinit, yval + yinit) = U_rep;
		}
	}
}

double distanceObs(PathPlanning *path, int x, int y)
{
	//printf("distanceObs\n");
	double dx, dy;
	int n = path->Obs.rows();
	//printf("obs length = %d", n);
	for (int i = 0; i < n; i++)
	{
		dx = x - path->Obs(i, 0);
		dy = y - path->Obs(i, 1);
		path->minObs(i, 0) = sqrt(dx * dx + dy * dy);
	}
	return (double)path->minObs.minCoeff();
}

void attractive_potential_field(CtrlStruct *ctrl, int goalx, int goaly)
{
	PathPlanning *path;
	path = ctrl->path;
	attractive_potential_field_reverse(ctrl);
	path->U.setZero(path->xlen, path->ylen);
	double rho_goal_square = 0.0;
	int xinit = 100;
	int yinit = 100;
	for (int xval = -xinit; xval < path->xlen - xinit; xval++)
	{
		for (int yval = -yinit; yval < path->ylen - yinit; yval++)
		{
			rho_goal_square = (xval - goalx) * (xval - goalx) + (yval - goaly) * (yval - goaly);
			path->U(xval + xinit, yval + yinit) = path->k_att * rho_goal_square;
		}
	}
	path->U = path->k_att / path->k_rep * path->U.eval() / path->U.maxCoeff();
	path->M = path->M.eval() + path->U;
	//printf("Mdim = %d x %d \n\r", path->M.rows(), path->M.cols());
}

void attractive_potential_field_reverse(CtrlStruct *ctrl)
{
	PathPlanning *path;
	path = ctrl->path;
	path->M = path->M.eval() - path->U;
}

/*void repulsive_opp_potential_field(CtrlStruct *ctrl)
{

	PathPlanning *path;
	path = ctrl->path;
	repulsive_opp_potential_field_reverse(ctrl);
	int yval, xval;

	int x1 = ctrl->opp_pos->x1 * 100;
	int y1 = ctrl->opp_pos->y1 * 100;
	int x2 = ctrl->opp_pos->x2 * 100;
	int y2 = ctrl->opp_pos->y2 * 100;
	int x3 = ctrl->opp_pos->x3 * 100;
	int y3 = ctrl->opp_pos->y3 * 100;

	x1 = 0;
	y1 = 0;
	x2 = 0;
	y2 = 0;
	x3 = 0;
	y3 = 0;

	double dobs1, dobs2, dobs3, U_rep1, U_rep2, U_rep3;
	for (xval = -100; xval < path->xlen - 100; xval++)
	{
		for (yval = -150; yval < path->ylen - 150; yval++)
		{

			dobs1 = sqrt((xval - x1) * (xval - x1) + (yval - y1) * (yval - y1));
			dobs2 = sqrt((xval - x2) * (xval - x2) + (yval - y2) * (yval - y2));
			dobs3 = sqrt((xval - x3) * (xval - x3) + (yval - y3) * (yval - y3));

			if (dobs1 == 0.0)
				U_rep1 = path->k_rep;
			else if (dobs1 < 2 * path->rho_zero)
				U_rep1 = path->k_rep * exp(-5 * dobs1 / (2 * path->rho_zero));
			else
				U_rep1 = 0;

			if (dobs2 == 0.0)
				U_rep2 = 2 * path->k_rep;
			else if (dobs2 < 2 * path->rho_zero)
				U_rep2 = path->k_rep * 2 * exp(-5 * dobs2 / (2 * path->rho_zero));
			else
				U_rep2 = 0;

			if (dobs3 == 0.0)
				U_rep3 = 2 * path->k_rep;
			else if (dobs3 < 2 * path->rho_zero)
				U_rep3 = path->k_rep * 2 * exp(-5 * dobs3 / (2 * path->rho_zero));
			else
				U_rep3 = 0;

			path->Opp(xval + 100, yval + 150) = U_rep1; //+ U_rep2 + U_rep3;
		}
	}
	path->Opp = path->Opp.eval() / path->Opp.maxCoeff();
	path->M = path->M.eval() + path->Opp;
} */

void repulsive_opp_potential_field_reverse(CtrlStruct *ctrl)
{
	PathPlanning *path;
	path = ctrl->path;
	path->M = path->M.eval() - path->Opp;
}

void solve_path(CtrlStruct *ctrl, int x, int y, int goalx, int goaly)
{
	PathPlanning *path;
	path = ctrl->path;
	path->traj.resize(1, 2);
	path->traj.setZero(1, 2);
	path->traj(0, 0) = (double)x;
	path->traj(0, 1) = (double)y;
	int idx = 0;
	double U_up, U_down, U_right, U_left, U_next;
	while (true)
	{
		// scan of the next position regarding the potential in the position
		// scan for X
		if (x == 100 || isinbacktrack(path, x + 1, y))
		{
			U_right = path->k_rep;
		}
		else
		{
			//printf("in the else \n\r");
			U_right = path->M(x + 1 + 100, y + 100);
		}

		if (x == -100 || isinbacktrack(path, x - 1, y))
		{
			U_left = path->k_rep;
		}
		else
		{
			//printf("in the else \n\r");
			U_left = path->M(x - 1 + 100, y + 100);
		}

		// scan for Y
		if (y == 100 || isinbacktrack(path, x, y + 1))
		{
			U_up = path->k_rep;
		}
		else
		{
			//printf("in the else \n\r");
			U_up = path->M(x + 100, y + 1 + 100);
		}
		if (y == -100 || isinbacktrack(path, x, y - 1))
		{
			U_down = path->k_rep;
		}
		else
		{
			//printf("in the else \n\r");
			U_down = path->M(x + 100, y - 1 + 100);
		}

		U_next = fmin(fmin(U_left, U_right), fmin(U_down, U_up));

		if (U_next == U_right)
		{
			x = x + 1;
		}
		else if (U_next == U_up)
		{
			y = y + 1;
		}
		else if (U_next == U_left)
		{
			x = x - 1;
		}
		else if (U_next == U_down)
		{
			y = y - 1;
		}
		else
		{
			break;
		}

		// if the new x and y is the near the goal then break
		if (sqrt((x - goalx) * (x - goalx) + (y - goaly) * (y - goaly)) <= 13)
		{
			x = goalx;
			y = goaly;
			path->traj.conservativeResize(idx + 1, 2);
			path->traj(idx, 0) = x;
			path->traj(idx, 1) = y;
			break;
		}

		idx++; // restart the while
		path->traj.conservativeResize(idx + 1, 2);
		path->traj(idx, 0) = x;
		path->traj(idx, 1) = y;
		//printf("PATH : posx(idx) = %f & posy(idx) = %f \n\r", path->traj(idx,0), path->traj(idx,1));
	}
	path->path_changed = 1;
}

int isinbacktrack(PathPlanning *path, int x, int y)
{
	int n = path->traj.rows();
	int xval, yval;
	for (int i = 0; i < n; i++)
	{
		int xval = (int)path->traj(i, 0);
		int yval = (int)path->traj(i, 1);
		if ((x == xval) && (y == yval))
		{
			return 1;
		}
	}
	return 0;
}
