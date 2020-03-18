#include "Odometry.hh"

Odometry::Odometry(CtrlStruct *theCtrlStruct)
{
    this->theCtrlStruct = theCtrlStruct;
}

void Odometry::Odometry_init()
{
    this->theCtrlStruct->theCtrlIn->odo_radius = 44.5 / 2000; //radius of the wheel in meters
    this->theCtrlStruct->theCtrlIn->l_odo_dist_prev = 0;
    this->theCtrlStruct->theCtrlIn->r_odo_dist_prev = 0;
    this->theCtrlStruct->theCtrlIn->x = 0;
    this->theCtrlStruct->theCtrlIn->y = 0;
    this->theCtrlStruct->theCtrlIn->theta = 0;
    this->theCtrlStruct->theCtrlIn->odo_tics_per_rot = 2048;
}

void Odometry::Odometry_start()
{
    pthread_create(&tr, NULL, &Odometry_update, this);
}

void *Odometry::Odometry_update(void *daOdometry){

}

void Odometry::update_rot(unsigned char *buffer){
    
}