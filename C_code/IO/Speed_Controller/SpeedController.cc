

#include "SpeedController.hh"
#include <pthread.h>
#include <chrono>
#include <wiringPiSPI.h>
#include <math.h>
#include <iostream>
#include "stdio.h"

SpeedController::SpeedController(CtrlStruct *theCtrlStruct, CAN0_Alternate *can0)
{
    this->can0 = can0;
    this->theCtrlStruct = theCtrlStruct;
}

void SpeedController::init_speed_controller(int i)
{
    speed_controller_active(i);
    double Ra = 7.1;
    double Kv = 4.3e-5;
    double J_rotor = 12e-7;
    double J_robot = 3 * 7.03125e-6;
    double J = J_rotor + J_robot;
    double tau_m = J / Kv;
    double kphi = 37.83e-3;
    double Kp = 3 * Ra * Kv / kphi;
    double Ki = Kp * ((Ra * Kv + kphi * Kp) / (J * Ra) - 3 / tau_m);
    double Current_max = 2; //0.78; // Ampere
    double secu = 0.95;
    double ratio = 7;

    this->theCtrlStruct->theUserStruct->samplingDE0 = 2000;
    this->theCtrlStruct->theUserStruct->tics = 2048;
    this->theCtrlStruct->theUserStruct->speed_kill = 0;

    this->theCtrlStruct->theUserStruct->theMotLeft->kp = Kp;
    this->theCtrlStruct->theUserStruct->theMotLeft->ki = Ki;
    this->theCtrlStruct->theUserStruct->theMotLeft->integral_error = 0;
    this->theCtrlStruct->theUserStruct->theMotLeft->status = 0;
    this->theCtrlStruct->theUserStruct->theMotLeft->Ra = Ra;
    this->theCtrlStruct->theUserStruct->theMotLeft->kphi = kphi;
    this->theCtrlStruct->theUserStruct->theMotLeft->t_p = 0;
    this->theCtrlStruct->theUserStruct->theMotLeft->ratio = 7;
    this->theCtrlStruct->theUserStruct->theMotLeft->upperCurrentLimit = Ra * Current_max;
    this->theCtrlStruct->theUserStruct->theMotLeft->lowerCurrentLimit = -Ra * Current_max;
    this->theCtrlStruct->theUserStruct->theMotLeft->upperVoltageLimit = 24 * secu;
    this->theCtrlStruct->theUserStruct->theMotLeft->lowerVoltageLimit = -24 * secu;

    this->theCtrlStruct->theUserStruct->theMotRight->kp = 0.115; //Kp;
    this->theCtrlStruct->theUserStruct->theMotRight->ki = 0.07; //Ki;
    this->theCtrlStruct->theUserStruct->theMotRight->integral_error = 0;
    this->theCtrlStruct->theUserStruct->theMotRight->status = 0;
    this->theCtrlStruct->theUserStruct->theMotRight->Ra = Ra;
    this->theCtrlStruct->theUserStruct->theMotRight->kphi = kphi;
    this->theCtrlStruct->theUserStruct->theMotRight->t_p = 0;
    this->theCtrlStruct->theUserStruct->theMotRight->ratio = 7;
    this->theCtrlStruct->theUserStruct->theMotRight->upperCurrentLimit = Ra * Current_max;
    this->theCtrlStruct->theUserStruct->theMotRight->lowerCurrentLimit = -Ra * Current_max;
    this->theCtrlStruct->theUserStruct->theMotRight->upperVoltageLimit = 24 * secu;
    this->theCtrlStruct->theUserStruct->theMotRight->lowerVoltageLimit = -24 * secu;
}

void SpeedController::speed_controller_active(int i)
{
    this->can0->CAN0ctrl_motor(i);
    this->theCtrlStruct->theUserStruct->speed_kill = !i;
}

void SpeedController::run_speed_controller()
{

    //run the thread here
    pthread_create(&tr, NULL, &updateLowCtrl, this);
}

void *SpeedController::updateLowCtrl(void *daSpeedController)
{

    auto start = std::chrono::steady_clock::now();
    unsigned char buffer[5];
    FILE *logFile = fopen("/home/pi/RobotCode/logFile.txt", "w");
    fprintf(logFile, "Rspeed Rref Lspeed Lref\r\n");

    while (((SpeedController *)daSpeedController)->theCtrlStruct->theUserStruct->speed_kill == 0)
    {
        ((SpeedController *)daSpeedController)->updateSpeed(buffer);
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
        double time_taken = (elapsed.count());
        //  printf("time taken sinds the controller is active: %f\r\n", time_taken / 1000);
        ((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->t = time_taken / 1000; //Temps utilisé pour mettre a jour les valeurs et appeler le speed controller
        ((SpeedController *)daSpeedController)->updateCmd();
        fprintf(logFile, "%0.1f %0.1f %0.1f %0.1f\r\n", ((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->r_wheel_speed, ((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->r_wheel_ref, ((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->l_wheel_speed, ((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->l_wheel_ref);
        //delay(50);
    }
    fclose(logFile);
}

void SpeedController::updateSpeed(unsigned char *buffer)
{

    //adresse des roues
    buffer[0] = 0x00;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    buffer[3] = 0x00;
    buffer[4] = 0x00;

    wiringPiSPIDataRW(0, buffer, 5);
    delay(10);

    this->theCtrlStruct->theCtrlIn->l_wheel_speed = (((double)(int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4])) * this->theCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (this->theCtrlStruct->theUserStruct->theMotLeft->ratio * this->theCtrlStruct->theUserStruct->tics);
    this->theCtrlStruct->theCtrlIn->r_wheel_speed = -(((double)(int16_t)((uint16_t)buffer[1] << 8 | (uint16_t)buffer[2])) * this->theCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (this->theCtrlStruct->theUserStruct->theMotRight->ratio * this->theCtrlStruct->theUserStruct->tics);

    printf(" l_wheel_speed %f", this->theCtrlStruct->theCtrlIn->l_wheel_speed);
    printf(" r_wheel_speed %f\n", this->theCtrlStruct->theCtrlIn->r_wheel_speed);
    //Mise a jour du pas de temps
}

void SpeedController::updateCmd()
{
    double omega_ref_l = this->theCtrlStruct->theCtrlIn->l_wheel_ref;
    double omega_ref_r = this->theCtrlStruct->theCtrlIn->r_wheel_ref;
    double r_wheel_speed = this->theCtrlStruct->theCtrlIn->r_wheel_speed;
    double l_wheel_speed = this->theCtrlStruct->theCtrlIn->l_wheel_speed;
    double t = this->theCtrlStruct->theCtrlIn->t;

    double cmd_l = this->PIController(this->theCtrlStruct->theUserStruct->theMotLeft, omega_ref_l, l_wheel_speed, t);
    double cmd_r = this->PIController(this->theCtrlStruct->theUserStruct->theMotRight, omega_ref_r, r_wheel_speed, t);

    this->theCtrlStruct->theCtrlOut->wheel_commands[L_ID] = cmd_l;
    this->theCtrlStruct->theCtrlOut->wheel_commands[R_ID] = cmd_r;

    printf(" l_wheel_command %f", this->theCtrlStruct->theCtrlOut->wheel_commands[L_ID]);
    printf(" r_wheel_command %f\n", this->theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);

    this->can0->CAN0pushPropDC(this->theCtrlStruct->theCtrlOut->wheel_commands[L_ID], this->theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
}

double SpeedController::PIController(MotStruct *theMot, double V_ref, double V_wheel_mes, double t)
{
    double e = V_ref - V_wheel_mes;
    double dt = t - theMot->t_p;
    double u = theMot->kp * e;

    if (!theMot->status) //The integral action is only done if there is no saturation of current.
    {
        theMot->integral_error += dt * e * theMot->ki;
    }
    u += theMot->integral_error;     // integral action
    u += theMot->kphi * V_wheel_mes; // back electromotive compensation
    theMot->status = saturation(theMot->upperCurrentLimit + theMot->kphi * V_wheel_mes, theMot->lowerCurrentLimit - theMot->kphi * V_wheel_mes, u);

    theMot->t_p = t;

    //OUTPUT
    return u * (100 / theMot->upperVoltageLimit);
}

int SpeedController::saturation(double upperLimit, double lowerLimit, double u)
{
    if (u > upperLimit)
    {
        u = upperLimit;
        return 1;
    }
    else if (u < lowerLimit)
    {
        u = lowerLimit;
        return 1;
    }
    else
        return 0;
}
