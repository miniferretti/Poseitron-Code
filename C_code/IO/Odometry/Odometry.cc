#include "IO/Odometry/Odometry.hh"

Odometry::Odometry(CtrlStruct *theCtrlStruct)
{
    this->theCtrlStruct = theCtrlStruct;
}

void Odometry::Odometry_init()
{
    unsigned char buffer[5];
    this->theCtrlStruct->robot->odo_radius = 1.5833 * 45.0 / 2000.0; //radius of the wheel in meters
    this->theCtrlStruct->theCtrlIn->l_odo_dist_prev = 0.0;
    this->theCtrlStruct->theCtrlIn->r_odo_dist_prev = 0.0;
    this->theCtrlStruct->rob_pos->x = 0.0;
    this->theCtrlStruct->rob_pos->x_prev = 0.0;
    this->theCtrlStruct->rob_pos->y = 0.0;
    this->theCtrlStruct->rob_pos->y_prev = 0.0;
    this->theCtrlStruct->rob_pos->dydx = 0;
    this->theCtrlStruct->rob_pos->dydx_prev = 0;
    this->theCtrlStruct->rob_pos->dist = 0;
    this->theCtrlStruct->rob_pos->dist_prev = 0;
    this->theCtrlStruct->rob_pos->ddist = 0;
    this->theCtrlStruct->rob_pos->ddist_flag = 0;
    this->theCtrlStruct->rob_pos->theta = 0.0;
    this->theCtrlStruct->rob_pos->t_used = 0;
    this->theCtrlStruct->rob_pos->dtheta = 0;
    this->theCtrlStruct->rob_pos->thetaref = 0;
    this->theCtrlStruct->rob_pos->dtheta_flag = 0;
    this->theCtrlStruct->rob_pos->kr = 0.07;
    this->theCtrlStruct->rob_pos->kl = 0.07;
    this->theCtrlStruct->rob_pos->ignore = 2;
    this->theCtrlStruct->rob_pos->covs = Eigen::MatrixXd::Zero(2, 2);
    this->theCtrlStruct->rob_pos->error = Eigen::MatrixXd::Zero(3, 3);
    this->theCtrlStruct->rob_pos->Dpf = Eigen::MatrixXd::Zero(3, 3);
    this->theCtrlStruct->rob_pos->Drlf = Eigen::MatrixXd::Zero(3, 2);
    this->theCtrlStruct->robot->odo_tics_per_rot = 2048.0;
    this->theCtrlStruct->robot->robot_width = 22.3 / 100.0; //value in meters
    this->theCtrlStruct->theUserStruct->Odo_kill = 0.0;
    this->logFile = fopen("/home/pi/Poseitron-Code/Data/LogFileOdometry.txt", "w");
    fprintf(this->logFile, "X Y Theta dr dl Time\r\n");
    this->update_rot(buffer);
}

void Odometry::Odometry_update()
{
    unsigned char buffer[5];
    double dr;
    double dl;
    double l = this->theCtrlStruct->robot->robot_width;
    double theta;
    double dtheta;
    double x;
    double dx;
    double y;
    double dy;

    if (this->theCtrlStruct->theUserStruct->Odo_kill == 0)
    {
        this->update_rot(buffer);
        dr = this->theCtrlStruct->theCtrlIn->r_odo_dist - this->theCtrlStruct->theCtrlIn->r_odo_dist_prev;
        dl = this->theCtrlStruct->theCtrlIn->l_odo_dist - this->theCtrlStruct->theCtrlIn->l_odo_dist_prev;
        x = this->theCtrlStruct->rob_pos->x;
        y = this->theCtrlStruct->rob_pos->y;
        theta = this->theCtrlStruct->rob_pos->theta;

        dx = ((dr + dl) / 2) * cos(theta + ((dr - dl) / (2 * l)));
        dy = ((dr + dl) / 2) * sin(theta + ((dr - dl) / (2 * l)));
        dtheta = ((dr - dl) / l);

        this->theCtrlStruct->rob_pos->x = x + dx;
        this->theCtrlStruct->rob_pos->y = y + dy;
        this->theCtrlStruct->rob_pos->theta = theta + dtheta;
        this->theCtrlStruct->rob_pos->dydx_prev = this->theCtrlStruct->rob_pos->dydx;

        /*   if (this->theCtrlStruct->rob_pos->theta < -M_PI)
        {
            this->theCtrlStruct->rob_pos->theta += 2 * M_PI;
        }

        if (this->theCtrlStruct->rob_pos->theta > M_PI)
        {
            this->theCtrlStruct->rob_pos->theta -= 2 * M_PI;
        } */

        if (dx != 0.0)
        {
            this->theCtrlStruct->rob_pos->dydx = dy / dx;
        }
        else
        {
            this->theCtrlStruct->rob_pos->dydx = 0;
        }

        // Part of the code for providing a reference angle and a delta angle based on it. Usefull for counting a number of turn of the robot.
        if (this->theCtrlStruct->rob_pos->dtheta_flag)
        {
            this->theCtrlStruct->rob_pos->dtheta = this->theCtrlStruct->rob_pos->theta - this->theCtrlStruct->rob_pos->thetaref;
        }
        if (this->theCtrlStruct->rob_pos->ddist_flag)
        {

            this->theCtrlStruct->rob_pos->ddist += sqrt((dx * dx) + (dy * dy));
        }

        //Calcule de l'erreur d'Odométrie, injection des valeurs dans les matrices.

        this->theCtrlStruct->rob_pos->Dpf << 1.0, 0.0, -((dr + dl) / 2) * sin(theta + (dtheta / 2)),
            0.0, 1.0, ((dr + dl) / 2) * cos(theta + (dtheta / 2)),
            0.0, 0.0, 1.0;

        this->theCtrlStruct->rob_pos->Drlf << (0.5 * cos(theta + (dtheta / 2)) - dtheta * 0.5 * sin(theta + (dtheta / 2))), (0.5 * cos(theta + (dtheta / 2)) + dtheta * 0.5 * sin(theta + (dtheta / 2))),
            (0.5 * sin(theta + (dtheta / 2)) + dtheta * 0.5 * cos(theta + (dtheta / 2))), (0.5 * sin(theta + (dtheta / 2)) - dtheta * 0.5 * cos(theta + (dtheta / 2))),
            2 / l, -2 / l;

        this->theCtrlStruct->rob_pos->covs << this->theCtrlStruct->rob_pos->kr * abs(dr), 0.0,
            0.0, this->theCtrlStruct->rob_pos->kl * abs(dl);

        this->theCtrlStruct->rob_pos->error = this->theCtrlStruct->rob_pos->Dpf * this->theCtrlStruct->rob_pos->error * this->theCtrlStruct->rob_pos->Dpf.transpose() + this->theCtrlStruct->rob_pos->Drlf * this->theCtrlStruct->rob_pos->covs * this->theCtrlStruct->rob_pos->Drlf.transpose();

        this->theCtrlStruct->rob_pos->t_used = this->theCtrlStruct->theCtrlIn->t;

        fprintf(this->logFile, "%f %f %f %f %f %f\r\n",
                this->theCtrlStruct->rob_pos->x,
                this->theCtrlStruct->rob_pos->y,
                this->theCtrlStruct->rob_pos->theta,
                dr,
                dl,
                this->theCtrlStruct->rob_pos->t_used);
    }
}

void Odometry::update_rot(unsigned char *buffer)
{

    //Buffer d'aquisition de la trace des roues.
    //Roue doite

    buffer[0] = 0x01;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    buffer[3] = 0x00;
    buffer[4] = 0x00;

    wiringPiSPIDataRW(0, buffer, 5);

    this->theCtrlStruct->theCtrlIn->r_odo_dist_prev = this->theCtrlStruct->theCtrlIn->r_odo_dist;
    this->theCtrlStruct->theCtrlIn->r_odo_dist = (((double)(int32_t)((uint32_t)buffer[1] << 24 | (uint32_t)buffer[2] << 16 | (uint32_t)buffer[3] << 8 | (uint32_t)buffer[4])) / this->theCtrlStruct->robot->odo_tics_per_rot) * this->theCtrlStruct->robot->odo_radius;

    //Roue gauche
    buffer[0] = 0x02;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    buffer[3] = 0x00;
    buffer[4] = 0x00;

    wiringPiSPIDataRW(0, buffer, 5);

    this->theCtrlStruct->theCtrlIn->l_odo_dist_prev = this->theCtrlStruct->theCtrlIn->l_odo_dist;
    this->theCtrlStruct->theCtrlIn->l_odo_dist = -(((double)(int32_t)((uint32_t)buffer[1] << 24 | (uint32_t)buffer[2] << 16 | (uint32_t)buffer[3] << 8 | (uint32_t)buffer[4])) / this->theCtrlStruct->robot->odo_tics_per_rot) * this->theCtrlStruct->robot->odo_radius;
}

void Odometry::Odometry_stop()
{
    this->theCtrlStruct->theUserStruct->Odo_kill = 1;
    fclose(this->logFile);
}

void Odometry::set_theta_ref()
{
    RobotPosition *odo_pos;
    odo_pos = this->theCtrlStruct->rob_pos;

    reset_theta_ref();

    odo_pos->dtheta_flag = 1;
    odo_pos->thetaref = odo_pos->theta;
}

void Odometry::reset_theta_ref()
{
    RobotPosition *odo_pos;
    odo_pos = this->theCtrlStruct->rob_pos;

    odo_pos->thetaref = 0;
    odo_pos->dtheta = 0;
    odo_pos->dtheta_flag = 0;
}

double Odometry::get_dtheta()
{
    RobotPosition *odo_pos;
    odo_pos = this->theCtrlStruct->rob_pos;

    return odo_pos->dtheta;
}

void Odometry::set_ddist()
{
    RobotPosition *odo_pos;
    odo_pos = this->theCtrlStruct->rob_pos;

    reset_theta_ref();

    odo_pos->ddist = 0;
    odo_pos->ddist_flag = 1;
}

void Odometry::reset_ddist()
{

    RobotPosition *odo_pos;
    odo_pos = this->theCtrlStruct->rob_pos;

    odo_pos->ddist = 0;
    odo_pos->ddist_flag = 0;
}

void Odometry::set_pos()
{
    RobotPosition *odo_pos;
    odo_pos = this->theCtrlStruct->rob_pos;

    odo_pos->x_prev = odo_pos->x;
    odo_pos->y_prev = odo_pos->y;
}

void Odometry::reset_pos()
{
    RobotPosition *odo_pos;
    odo_pos = this->theCtrlStruct->rob_pos;

    odo_pos->x_prev = 0;
    odo_pos->y_prev = 0;
}

double Odometry::get_ddist()
{
    RobotPosition *odo_pos;
    odo_pos = this->theCtrlStruct->rob_pos;

    return odo_pos->ddist;
}

void Odometry::reset_odometry()
{
    CtrlIn *inputs = this->theCtrlStruct->theCtrlIn;
    RobotPosition *odo_pos = this->theCtrlStruct->rob_pos;

    odo_pos->x = 0;
    odo_pos->x_prev = 0;
    odo_pos->y = 0;
    odo_pos->y_prev = 0;
    odo_pos->dist = 0;
    odo_pos->dist_prev = 0;
    odo_pos->ddist = 0;
    odo_pos->ddist_flag = 0;
    odo_pos->theta = 0;
    odo_pos->thetaref = 0;
    odo_pos->dtheta = 0;
    odo_pos->dtheta_flag = 0;
    odo_pos->t_used = inputs->t;
    odo_pos->error << 0, 0, 0, 0, 0, 0, 0, 0, 0;
}