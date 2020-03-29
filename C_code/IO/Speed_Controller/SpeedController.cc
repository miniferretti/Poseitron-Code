

#include "SpeedController.hh"
#include <pthread.h>
#include <chrono>
#include <wiringPiSPI.h>
#include <math.h>
#include <iostream>
#include "stdio.h"

SpeedController::SpeedController(CtrlStruct *theCtrlStruct, CAN0_Alternate *can0, pthread_mutex_t *theMutex)
{
    this->can0 = can0;
    this->theCtrlStruct = theCtrlStruct;
    this->theMutex = theMutex;
}

void SpeedController::init_speed_controller(int i)
{
    this->speed_controller_active(i);
    double Ra = 7.1;
    double Kv = 4.3e-5;
    double J_rotor = 12e-7;
    double J_robot = 3 * 7.03125e-6;
    double J = J_rotor + J_robot;
    double tau_m = J / Kv;
    double kphi = 37.83e-3;
    double Kp = 3 * Ra * Kv / kphi;
    double Ki = Kp * ((Ra * Kv + kphi * Kp) / (J * Ra) - 3 / tau_m);
    double Current_max = 0.78; //0.78; // Ampere
    double secu = 0.95;
    double ratio = 7;

    this->theCtrlStruct->theUserStruct->samplingDE0 = 2000;
    this->theCtrlStruct->theUserStruct->tics = 2048;
    this->theCtrlStruct->theUserStruct->speed_kill = 0;

    this->theCtrlStruct->theUserStruct->theMotLeft->kp = 0.05; //Kp;
    this->theCtrlStruct->theUserStruct->theMotLeft->ki = 0.6;  // valeur a modifier si besoins est...
    this->theCtrlStruct->theUserStruct->theMotLeft->kd = 0.0004;
    this->theCtrlStruct->theUserStruct->theMotLeft->integral_error = 0;
    this->theCtrlStruct->theUserStruct->theMotLeft->status = 0;
    this->theCtrlStruct->theUserStruct->theMotLeft->Ra = Ra;
    this->theCtrlStruct->theUserStruct->theMotLeft->kphi = kphi;
    this->theCtrlStruct->theUserStruct->theMotLeft->t_p = 0;
    this->theCtrlStruct->theUserStruct->theMotLeft->ratio = ratio;
    this->theCtrlStruct->theUserStruct->theMotLeft->upperCurrentLimit = Ra * Current_max;
    this->theCtrlStruct->theUserStruct->theMotLeft->lowerCurrentLimit = -Ra * Current_max;
    this->theCtrlStruct->theUserStruct->theMotLeft->upperVoltageLimit = 24 * secu;
    this->theCtrlStruct->theUserStruct->theMotLeft->lowerVoltageLimit = -24 * secu;

    this->theCtrlStruct->theUserStruct->theMotRight->kp = 0.025; //Kp;
    this->theCtrlStruct->theUserStruct->theMotRight->ki = 0.5;   //Ki;
    this->theCtrlStruct->theUserStruct->theMotRight->kd = 0.0005;
    this->theCtrlStruct->theUserStruct->theMotRight->integral_error = 0;
    this->theCtrlStruct->theUserStruct->theMotRight->status = 0;
    this->theCtrlStruct->theUserStruct->theMotRight->Ra = Ra;
    this->theCtrlStruct->theUserStruct->theMotRight->kphi = kphi;
    this->theCtrlStruct->theUserStruct->theMotRight->t_p = 0;
    this->theCtrlStruct->theUserStruct->theMotRight->ratio = ratio;
    this->theCtrlStruct->theUserStruct->theMotRight->upperCurrentLimit = Ra * Current_max;
    this->theCtrlStruct->theUserStruct->theMotRight->lowerCurrentLimit = -Ra * Current_max;
    this->theCtrlStruct->theUserStruct->theMotRight->upperVoltageLimit = 24 * secu;
    this->theCtrlStruct->theUserStruct->theMotRight->lowerVoltageLimit = -24 * secu;

    for (int i = 0; i < MVG_LENG; i++)
    {
        this->avgL[i] = 0;
        this->avgR[i] = 0;
    }
}

void SpeedController::speed_controller_active(int i)
{
    this->theCtrlStruct->theUserStruct->speed_kill = !i;
    if (i == 0)
    {
        pthread_join(this->tr, NULL);
    }
    this->can0->CAN0ctrl_motor(i);
}

void SpeedController::run_speed_controller()
{
    pthread_create(&tr, NULL, &updateLowCtrl, this); //Lancement du thread
}

void *SpeedController::updateLowCtrl(void *daSpeedController)
{

    auto start = std::chrono::steady_clock::now();

    double time_taken;
    unsigned char buffer[5];
    FILE *logFile = fopen("/home/pi/RobotCode/logFileSpeed.txt", "w"); //Ouverture et ecrasement du fichier log pour l'historique des vitesses (pour Matlab par exemple...)
    fprintf(logFile, "Rspeed Rref Lspeed Lref Time\r\n");

    while (((SpeedController *)daSpeedController)->theCtrlStruct->theUserStruct->speed_kill == 0)
    {

        ((SpeedController *)daSpeedController)->updateSpeed(buffer);
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
        time_taken = (elapsed.count());
        ((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->t = time_taken / 1000.0; //Temps utilisé pour mettre a jour les valeurs et appeler le speed controller
        ((SpeedController *)daSpeedController)->updateCmd();
        fprintf(logFile, "%0.1f %0.1f %0.1f %0.1f %f\r\n",
                -((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->r_wheel_speed,
                ((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->r_wheel_ref,
                ((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->l_wheel_speed,
                ((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->l_wheel_ref,
                ((SpeedController *)daSpeedController)->theCtrlStruct->theCtrlIn->t);
        usleep(10000);
    }
    fclose(logFile);
}

void SpeedController::updateSpeed(unsigned char *buffer)
{
    pthread_mutex_lock(this->theMutex);
    //adresse des roues
    buffer[0] = 0x00;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    buffer[3] = 0x00;
    buffer[4] = 0x00;

    wiringPiSPIDataRW(0, buffer, 5);

    /*  double speedL = -(((double)(int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4])) * this->theCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (this->theCtrlStruct->theUserStruct->theMotLeft->ratio * this->theCtrlStruct->theUserStruct->tics);
    double speedR = -(((double)(int16_t)((uint16_t)buffer[1] << 8 | (uint16_t)buffer[2])) * this->theCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (this->theCtrlStruct->theUserStruct->theMotRight->ratio * this->theCtrlStruct->theUserStruct->tics);

    this->theCtrlStruct->theCtrlIn->l_wheel_speed = this->Moving_Average(speedL, this->avgL, MVG_LENG);
    this->theCtrlStruct->theCtrlIn->r_wheel_speed = this->Moving_Average(speedR, this->avgR, MVG_LENG);*/

    this->theCtrlStruct->theCtrlIn->l_wheel_speed = -(((double)(int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4])) * this->theCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (this->theCtrlStruct->theUserStruct->theMotLeft->ratio * this->theCtrlStruct->theUserStruct->tics);
    this->theCtrlStruct->theCtrlIn->r_wheel_speed = -(((double)(int16_t)((uint16_t)buffer[1] << 8 | (uint16_t)buffer[2])) * this->theCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (this->theCtrlStruct->theUserStruct->theMotRight->ratio * this->theCtrlStruct->theUserStruct->tics);

    printf(" l_wheel_speed %f", this->theCtrlStruct->theCtrlIn->l_wheel_speed);
    printf(" r_wheel_speed %f", -this->theCtrlStruct->theCtrlIn->r_wheel_speed);
    pthread_mutex_unlock(this->theMutex);
}

void SpeedController::updateCmd()
{

    double omega_ref_l = this->theCtrlStruct->theCtrlIn->l_wheel_ref;
    double omega_ref_r = -this->theCtrlStruct->theCtrlIn->r_wheel_ref;
    double r_wheel_speed = this->theCtrlStruct->theCtrlIn->r_wheel_speed;
    double l_wheel_speed = this->theCtrlStruct->theCtrlIn->l_wheel_speed;
    double t = this->theCtrlStruct->theCtrlIn->t;

    double cmd_l = this->PIController(this->theCtrlStruct->theUserStruct->theMotLeft, omega_ref_l, l_wheel_speed, t);
    double cmd_r = this->PIController(this->theCtrlStruct->theUserStruct->theMotRight, omega_ref_r, r_wheel_speed, t);

    this->theCtrlStruct->theCtrlOut->wheel_commands[L_ID] = cmd_l;
    this->theCtrlStruct->theCtrlOut->wheel_commands[R_ID] = cmd_r;

    printf(" l_wheel_command %f", this->theCtrlStruct->theCtrlOut->wheel_commands[L_ID]);
    printf(" r_wheel_command %f\n", -this->theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);

    pthread_mutex_lock(this->theMutex);
    this->can0->CAN0pushPropDC(this->theCtrlStruct->theCtrlOut->wheel_commands[L_ID], this->theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
    pthread_mutex_unlock(this->theMutex);
}

double SpeedController::PIController(MotStruct *theMot, double V_ref, double V_wheel_mes, double t)
{
    double e = V_ref - V_wheel_mes;
    double dt = t - theMot->t_p;
    double u = theMot->kp * e;

    printf(" dt = %f\r\n", dt);

    if (!theMot->status) //The integral action is only done if there is no saturation of current.
    {
        theMot->integral_error += dt * e;
    }
    // integral action
    u += theMot->integral_error * theMot->ki;
    u += theMot->kphi * V_wheel_mes; // back electromotive compensation
    u += theMot->kd * e / dt;
    theMot->status = saturation(theMot->upperVoltageLimit, theMot->lowerVoltageLimit, &u);

    theMot->t_p = t;

    //OUTPUT
    return u * (100 / theMot->upperVoltageLimit) * 0.98;
}

int SpeedController::saturation(double upperLimit, double lowerLimit, double *u)
{
    if (*u > upperLimit)
    {
        *u = upperLimit;
        return 1;
    }
    else if (*u < lowerLimit)
    {
        *u = lowerLimit;
        return 1;
    }
    else
        return 0;
}

double SpeedController::Moving_Average(double speed, double *buff, int leng)
{

    int count = 0;
    double sum = 0;

    for (int i = 0; i < leng; i++)
    {
        if (i == leng - 1)
        {
            buff[leng - 1 - i] = speed;
            sum += buff[leng - 1 - i];
        }
        else
        {
            if (buff[leng - 1 - i] == 0)
            {
                buff[leng - 1 - i] = speed;
            }
            else
            {
                buff[leng - 1 - i] = buff[leng - 1 - i - 1];
            }
            sum += buff[leng - 1 - i];
        }
    }

    return (sum / ((double)leng));
}
