#include "path_follow.h"

int path_follow(CtrlStruct *ctrl)
{
    // Variable declaration
    PathFollow *follower;
    RobotPosition *rob_pos;
    CtrlIn *inputs;
    PathPlanning *path;
    RobotParameters *robot_param;
    double dx, dy, real_alpha, v, omega, omega_sat, speed_sat;

    // variables initialization
    follower = ctrl->follower;
    rob_pos = ctrl->rob_pos;
    inputs = ctrl->theCtrlIn;
    path = ctrl->path;
    robot_param = ctrl->robot;
    // Code
    if (path->path_changed == 1)
    {
        follower->count = 0;
        follower->next = 1;
        if (path->traj.rows() <= 1)
        {
            printf("\n\rERROR : Path not well defined (size %dx%d) !!!\n\r", (int)path->traj.rows(), (int)path->traj.cols());
            ctrl->main_states = STOP_STATE;
        }
    }

    if (follower->next == 1)
    {
        if (follower->last == 0)
        {
            follower->count += 5;
            follower->next = 0;
            if (path->path_changed != 1)
            {
                printf(">>> path(%d) (path length %d) \n\r", (int)follower->count, (int)path->traj.rows());
            }
        }
        else
            follower->next = 0;
    }
    dx = path->traj(follower->count, 0) / 100 - rob_pos->x; // attention le path est en cm.
    dy = path->traj(follower->count, 1) / 100 - rob_pos->y;

    /* Mise en place des vitesses de reference sur base d'un feedback control
     * v = k_rho * rho
     * w = k_alpha * alpha + k_beta * beta
     * */
    follower->rho = sqrt(dx * dx + dy * dy);
    follower->alpha = atan(tan(-rob_pos->theta + atan2(dy, dx)));
    follower->beta = -atan(dy / dx);

    //printf("alpha = %f (theta = %f)\n\r",follower->alpha, rob_pos->theta);
    /* On fait en sorte que si on part dans le sens inverse, on reste dans le sens inverse 
     * pdt toute la trajectoire courante et resp. dans le sens direct.
     * 
     * */
    if (path->path_changed == 1)
    {
        real_alpha = -rob_pos->theta + atan2(dy, dx);
        if (real_alpha < -M_PI)
        {
            real_alpha += 2 * M_PI;
        }
        else if (real_alpha > M_PI)
        {
            real_alpha -= 2 * M_PI;
        }
        if (real_alpha > -M_PI / 2 && real_alpha < M_PI / 2)
        {
            path->path_changed = 0;
            follower->v_changed = 1;
            printf("\n/////////////////////////////////////\n\r");
            printf("Start of middle control : path follow\n\n");
            printf(">>> path changed : go forward (alpha = %f)\n", real_alpha * 180 / M_PI);
        }
        else
        {
            path->path_changed = 0;
            follower->v_changed = -1;
            printf("\n/////////////////////////////////////\n\r");
            printf("Start of middle control : path follow\n\n");
            printf(">>> path changed : go backward (alpha = %f)\n", real_alpha * 180 / M_PI);
        }
    }

    //  v = follower->v_changed * follower->Krho * follower->rho;
    v = follower->v_changed * follower->speed_sat;
    omega = follower->Kalpha * follower->alpha + follower->Kbeta * follower->beta;

    /* Saturation des vitesses.
     * On garde les vitesses dans des valeurs raisonables et 
     * et on fait en sorte que le ratio entre les deux vitesses soit conservé.
     * */
    omega_sat = follower->omega_sat;
    speed_sat = follower->speed_sat;
    if (speed_sat < v || omega_sat < omega)
    {
        if (v > omega)
        {
            omega = omega * abs(speed_sat / v);
            v = speed_sat;
        }
        else
        {
            v = v * abs(omega_sat / omega);
            omega = omega_sat;
        }
    }
    if (-speed_sat > v || -omega_sat > omega)
    {
        if (v < omega)
        {
            omega = omega * abs(speed_sat / v);
            v = -speed_sat;
        }
        else
        {
            v = v * abs(omega_sat / omega);
            omega = -omega_sat;
        }
    }
    //set_plot(follower->alpha/M_PI, "alpha");
    //set_plot(v/speed_sat, "v ref");
    //set_plot(omega/omega_sat, "w ref");

    /* Transformation vitesse lineaire et angulaire 
     * en vitesse de roue droite et gauche.
     * */
    //printf("omega = %f\n\r", omega);
    inputs->r_wheel_ref = ((v + omega * robot_param->wheel_dist / 2) / robot_param->wheel_rad) * ctrl->theUserStruct->theMotRight->compensation_factor;
    inputs->l_wheel_ref = ((v - omega * robot_param->wheel_dist / 2) / robot_param->wheel_rad) * ctrl->theUserStruct->theMotLeft->compensation_factor;

    /* Choix du path suivant : 
     * si rho <= distance q du point de reference au temp t*
     * on change de point de reference 
     * sinon on garde ce point de reference
    */
    if (follower->rho <= follower->rhoLimit)
    {
        follower->next = 1;
        if ((follower->count + 1 + 5) > path->traj.rows())
        {
            if (follower->last == 1)
            {
                follower->last = 0;
                return 1;
            }
            else
            {
                follower->count = path->traj.rows() - 1;
                follower->last = 1;
                return 0;
            }
        }
        else
            return 0;
    }
    return 0;
}