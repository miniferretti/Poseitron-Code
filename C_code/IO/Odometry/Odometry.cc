#include "IO/Odometry/Odometry.hh"

Odometry::Odometry(CtrlStruct *theCtrlStruct)
{
    this->theCtrlStruct = theCtrlStruct;
}

void Odometry::Odometry_init()
{
    unsigned char buffer[5];
    this->theCtrlStruct->robot->odo_radius = 45.0 / 2000.0; //radius of the wheel in meters
    this->theCtrlStruct->theCtrlIn->l_odo_dist_prev = 0.0;
    this->theCtrlStruct->theCtrlIn->r_odo_dist_prev = 0.0;
    this->theCtrlStruct->rob_pos->x = 0.0;
    this->theCtrlStruct->rob_pos->y = 0.0;
    this->theCtrlStruct->rob_pos->theta = 0.0;
    this->theCtrlStruct->robot->odo_tics_per_rot = 2048.0;
    this->theCtrlStruct->robot->robot_width = 22.3 / 100.0; //value in meters
    this->theCtrlStruct->theUserStruct->Odo_kill = 0.0;
    this->logFile = fopen("/home/pi/RobotCode/LogFileOdometry.txt", "w");
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
    double x;
    double y;

    if (this->theCtrlStruct->theUserStruct->Odo_kill == 0)
    {
        this->update_rot(buffer);
        dr = this->theCtrlStruct->theCtrlIn->r_odo_dist - this->theCtrlStruct->theCtrlIn->r_odo_dist_prev;
        dl = this->theCtrlStruct->theCtrlIn->l_odo_dist - this->theCtrlStruct->theCtrlIn->l_odo_dist_prev;
        x = this->theCtrlStruct->rob_pos->x;
        y = this->theCtrlStruct->rob_pos->y;
        theta = this->theCtrlStruct->rob_pos->theta;

        this->theCtrlStruct->rob_pos->x = x + ((dr + dl) / 2) * cos(theta + ((dr - dl) / (2 * l)));
        this->theCtrlStruct->rob_pos->y = y + ((dr + dl) / 2) * sin(theta + ((dr - dl) / (2 * l)));
        this->theCtrlStruct->rob_pos->theta = theta + ((dr - dl) / l);
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

void Odometry::set_thetaRef(double ref){
    this->theCtrlStruct->rob_pos->thetaref = ref;
}