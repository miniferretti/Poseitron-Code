

#include "SpeedController.hh"
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

    this->theCtrlStruct->theUserStruct->theMotLeft->kp = 0.04; //Kp;
    this->theCtrlStruct->theUserStruct->theMotLeft->ki = 0.7;  // valeur a modifier si besoins est...
    this->theCtrlStruct->theUserStruct->theMotLeft->kd = 0.00004;
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
    this->theCtrlStruct->theUserStruct->theMotLeft->compensation_factor = 1;
    this->theCtrlStruct->theUserStruct->theMotLeft->ki_flag = 0;

    this->theCtrlStruct->theUserStruct->theMotRight->kp = 0.04; //Kp;
    this->theCtrlStruct->theUserStruct->theMotRight->ki = 0.7;  //Ki;
    this->theCtrlStruct->theUserStruct->theMotRight->kd = 0.00004;
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
    this->theCtrlStruct->theUserStruct->theMotRight->compensation_factor = 1;
    this->theCtrlStruct->theUserStruct->theMotRight->ki_flag = 0;
    slave_speed_right = 0;
    slave_speed_left = 0;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;
    kp_left = 0;
    slave = 2;
    fromlen = sizeof(struct sockaddr_in);

    for (int i = 0; i < MVG_LENG; i++)
    {
        this->avgL[i] = 0;
        this->avgR[i] = 0;
    }
    this->logFile = fopen("/home/pi/Poseitron-Code/Data/logFileSpeed.txt", "w");
    this->PIDFile = fopen("/home/pi/Poseitron-Code/Data/PID.txt", "r");

    fprintf(logFile, "Rspeed Rref Lspeed Lref Time\r\n");

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
    {
        printf("Failed to create UDP server socket\r\n");
    }

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(UDP_PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    //  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    int rc = bind(sock, (const struct sockaddr *)&servaddr, sizeof(servaddr));

    if (rc == -1)
    {
        printf("Failed to bind \r\n");
        close(sock);
    }
    else
    {
        printf("Bind UDP socket succes !\r\n");
    }
}

void SpeedController::speed_controller_active(int i)
{
    this->theCtrlStruct->theUserStruct->speed_kill = !i;
    if (i == 0)
    {
        fclose(logFile);
    }
    this->can0->CAN0ctrl_motor(i);
}

void SpeedController::updateLowCtrl()
{

    unsigned char buffer[5];
    double speeds[5];
    unsigned char buf[44];
    /*   unsigned char buf1[4];
    unsigned char buf2[4];
    unsigned char buf3[4];
    unsigned char buf4[4];
    unsigned char buf5[4];
    unsigned char buf6[4];
    unsigned char buf7[4];
    unsigned char buf8[4]; */
    int n;

    if (this->theCtrlStruct->theUserStruct->speed_kill == 0)
    {
        //this->update_PID();
        this->updateSpeed(buffer);
        this->updateCmd();
        fprintf(logFile, "%0.1f %0.1f %0.1f %0.1f %f\n",
                -this->theCtrlStruct->theCtrlIn->r_wheel_speed,
                this->theCtrlStruct->theCtrlIn->r_wheel_ref,
                this->theCtrlStruct->theCtrlIn->l_wheel_speed,
                this->theCtrlStruct->theCtrlIn->l_wheel_ref,
                this->theCtrlStruct->theCtrlIn->t);

        speeds[0] = -this->theCtrlStruct->theCtrlIn->r_wheel_speed;
        speeds[1] = this->theCtrlStruct->theCtrlIn->r_wheel_ref;
        speeds[2] = this->theCtrlStruct->theCtrlIn->l_wheel_speed;
        speeds[3] = this->theCtrlStruct->theCtrlIn->l_wheel_ref;
        speeds[4] = this->theCtrlStruct->theCtrlIn->t;

        //  printf("size of the speed array = %d\r\n", sizeof(speeds));

        n = recvfrom(sock, (char *)buf, 44, MSG_DONTWAIT, (struct sockaddr *)&from, &fromlen);
        if (n > -1)
        {

            memcpy(&kp_left, &buf[0], sizeof(kp_left));
            memcpy(&ki_left, &buf[4], sizeof(kp_left));
            memcpy(&kd_left, &buf[8], sizeof(kp_left));
            memcpy(&kp_right, &buf[12], sizeof(kp_right));
            memcpy(&ki_right, &buf[16], sizeof(ki_right));
            memcpy(&kd_right, &buf[20], sizeof(kd_right));
            memcpy(&correction_factor_left, &buf[24], sizeof(correction_factor_left));
            memcpy(&correction_factor_right, &buf[28], sizeof(correction_factor_right));
            memcpy(&slave_speed_left, &buf[32], sizeof(slave_speed_left));
            memcpy(&slave_speed_right, &buf[36], sizeof(slave_speed_right));
            memcpy(&slave, &buf[40], sizeof(slave));

            this->theCtrlStruct->theUserStruct->theMotLeft->kp = kp_left; //Kp;
            if (float(this->theCtrlStruct->theUserStruct->theMotLeft->ki) != ki_left)
            {
                this->theCtrlStruct->theUserStruct->theMotLeft->ki_flag = 1;
            }
            else
            {
                this->theCtrlStruct->theUserStruct->theMotLeft->ki_flag = 0;
            }
            this->theCtrlStruct->theUserStruct->theMotLeft->ki = ki_left; // valeur a modifier si besoins est...
            this->theCtrlStruct->theUserStruct->theMotLeft->kd = kd_left;

            this->theCtrlStruct->theUserStruct->theMotRight->kp = kp_right; //Kp;
            if (float(this->theCtrlStruct->theUserStruct->theMotRight->ki) != ki_right)
            {
                this->theCtrlStruct->theUserStruct->theMotRight->ki_flag = 1;
            }
            else
            {
                this->theCtrlStruct->theUserStruct->theMotRight->ki_flag = 0;
            }
            this->theCtrlStruct->theUserStruct->theMotRight->ki = ki_right; //Ki;
            this->theCtrlStruct->theUserStruct->theMotRight->kd = kd_right;

            if (slave == 1)
            {
                slave_previous_state = this->theCtrlStruct->main_states;
                this->theCtrlStruct->main_states = SlAVE_STATE;
            }
            else if (slave == 0)
            {
                if (slave_previous_state != 0)
                {
                    this->theCtrlStruct->main_states = slave_previous_state;
                }
            }

            if (this->theCtrlStruct->main_states == SlAVE_STATE)
            {
                set_speed(slave_speed_left, slave_speed_right);
            }

            this->theCtrlStruct->theUserStruct->theMotLeft->compensation_factor = correction_factor_left;
            this->theCtrlStruct->theUserStruct->theMotRight->compensation_factor = correction_factor_right;

            //  printf("Yep data recieved requested\r\n");
            n = sendto(sock, speeds, sizeof(speeds), 0, (struct sockaddr *)&from, fromlen);
        }
        else
        {
            // printf("No data has been requested by the pyhton code\r\n");
        }
        printf("The value send is %f %f %f %f %f %f\r\n", kp_left, ki_left, kd_left, kp_right, ki_right, kd_right);
    }
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

    /*  double speedL = -(((double)(int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4])) * this->theCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (this->theCtrlStruct->theUserStruct->theMotLeft->ratio * this->theCtrlStruct->theUserStruct->tics);
    double speedR = -(((double)(int16_t)((uint16_t)buffer[1] << 8 | (uint16_t)buffer[2])) * this->theCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (this->theCtrlStruct->theUserStruct->theMotRight->ratio * this->theCtrlStruct->theUserStruct->tics);

    this->theCtrlStruct->theCtrlIn->l_wheel_speed = this->Moving_Average(speedL, this->avgL, MVG_LENG);
    this->theCtrlStruct->theCtrlIn->r_wheel_speed = this->Moving_Average(speedR, this->avgR, MVG_LENG);*/

    this->theCtrlStruct->theCtrlIn->l_wheel_speed = -(((double)(int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4])) * this->theCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (this->theCtrlStruct->theUserStruct->theMotLeft->ratio * this->theCtrlStruct->theUserStruct->tics);
    this->theCtrlStruct->theCtrlIn->r_wheel_speed = -(((double)(int16_t)((uint16_t)buffer[1] << 8 | (uint16_t)buffer[2])) * this->theCtrlStruct->theUserStruct->samplingDE0) * 2 * M_PI / (this->theCtrlStruct->theUserStruct->theMotRight->ratio * this->theCtrlStruct->theUserStruct->tics);

    //  printf(" l_wheel_speed %f", this->theCtrlStruct->theCtrlIn->l_wheel_speed);
    // printf(" r_wheel_speed %f\r\n", -this->theCtrlStruct->theCtrlIn->r_wheel_speed);
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

    // printf(" l_wheel_command %f", this->theCtrlStruct->theCtrlOut->wheel_commands[L_ID]);
    // printf(" r_wheel_command %f\n", -this->theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);

    this->can0->CAN0pushPropDC(this->theCtrlStruct->theCtrlOut->wheel_commands[L_ID], this->theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
    this->theCtrlStruct->theCtrlIn->sens_flag = this->can0->getDistance(1, this->theCtrlStruct->theCtrlIn->sens_array_front);
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
    if (theMot->ki_flag == 1)
    {
        theMot->integral_error = 0;
    }
    // integral action
    u += theMot->integral_error * theMot->ki;
    //  u += theMot->kphi * V_wheel_mes; // back electromotive compensation
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

void SpeedController::set_speed(double left, double right)
{
    this->theCtrlStruct->theCtrlIn->l_wheel_ref = left * this->theCtrlStruct->theUserStruct->theMotLeft->compensation_factor;
    this->theCtrlStruct->theCtrlIn->r_wheel_ref = right * this->theCtrlStruct->theUserStruct->theMotRight->compensation_factor;
}

void SpeedController::Speed_controller_stop()
{
    this->theCtrlStruct->theUserStruct->speed_kill = 1;

    fclose(this->logFile);

    this->can0->CAN0ctrl_motor(0);
}
