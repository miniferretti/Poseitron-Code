///////////////////////////////////////////////////////////////////////
//
// Written by: Eleonore Masarweh
// 
//
///////////////////////////////////////////////////////////////////////


//
// Library used to process the different proximity sensors data 
// in order to locate the oponnnent
//


#include <math.h>
#include "obstacle_processing.h"
#include "CtrlStruct.hh"
#include "ctrl_io.h"

void vue_and_map(CtrlStruct* theCtrlStruct){
    // First step : fill the structure in since sensor_array is changing along with I2C freq
    // and it is not now if these functions 
    theCtrlStruct->obstacles->sensor_signals[0] = theCtrlStruct->theCtrlIn->sens_array_front[0]; 
    theCtrlStruct->obstacles->sensor_signals[1] = theCtrlStruct->theCtrlIn->sens_array_front[1]; 
    theCtrlStruct->obstacles->sensor_signals[2] = theCtrlStruct->theCtrlIn->sens_array_front[2];
    theCtrlStruct->obstacles->sensor_signals[3] = theCtrlStruct->theCtrlIn->sens_array_front[3]; 
    theCtrlStruct->obstacles->sensor_signals[4] = theCtrlStruct->theCtrlIn->sens_array_front[4];

    theCtrlStruct->obstacles->sensor_signals[5] = theCtrlStruct->theCtrlIn->sens_array_back[0]; 
    theCtrlStruct->obstacles->sensor_signals[6] = theCtrlStruct->theCtrlIn->sens_array_back[1];  
    theCtrlStruct->obstacles->sensor_signals[7] = theCtrlStruct->theCtrlIn->sens_array_back[2];
    theCtrlStruct->obstacles->sensor_signals[8] = theCtrlStruct->theCtrlIn->sens_array_back[3];   
    theCtrlStruct->obstacles->sensor_signals[9] = theCtrlStruct->theCtrlIn->sens_array_back[4];

    // Second step : apply the correction to sensors that are not aligned with center of the robot
    // Angle from normal of side : 30 degrees (pi/6)
    sensorwise_crossed_correction(1, theCtrlStruct); 
    sensorwise_crossed_correction(3, theCtrlStruct); 
    sensorwise_crossed_correction(6, theCtrlStruct); 
    sensorwise_crossed_correction(8, theCtrlStruct);

    // Third step : fill in the vision vectors, 360 positions representing the angles 
    sensorwise_direct(0, theCtrlStruct); 
    sensorwise_direct(1, theCtrlStruct); 
    sensorwise_direct(2, theCtrlStruct); 
    sensorwise_direct(3, theCtrlStruct); 
    sensorwise_direct(4, theCtrlStruct); 
    sensorwise_direct(5, theCtrlStruct); 
    sensorwise_direct(6, theCtrlStruct);
    sensorwise_direct(7, theCtrlStruct); 
    sensorwise_direct(8, theCtrlStruct); 
    sensorwise_direct(9, theCtrlStruct); 

    // Fourth step : write the distances between center of the robot and obstacles in the
    // x_360 and y_360 vectors (each of the elements corresponds to the obstacle seen at 
    // the corresponding angle in the vision vector) 
    // This step takes the orientation of the robot in account and rotate the vision vector
    // to fit the seen map with the real one.  
    locate_obstacles(theCtrlStruct); 

    // Fifth step : write reliefs in the map at the seen place, taking robot position in account
    map_obstacles(theCtrlStruct); 

}

void sensorwise_crossed_correction(int signal_nmbr, CtrlStruct *theCtrlStruct){

    double signal = theCtrlStruct->obstacles->sensor_signals[signal_nmbr]; 
    double sensor_spacing = theCtrlStruct->obstacles->sensor_spacing; 
    double robot_side = theCtrlStruct->obstacles->robot_side; 

    double distance; 
    double x; 
    double y; 
    double angle_real; 
    double angle_degree;  
    int angle; 

        if(signal != 0){
            x = signal*cos(60) - sensor_spacing; 
            y = signal*sin(60) + robot_side; 

            angle_real = atan(x/y); 
            angle_degree = 180/M_PI * angle_real; 
            angle = round(angle_degree); 

            distance = sqrt(x^2+y^2) - (robot_side/2) * 1/cos(angle_real);
        }

    theCtrlStruct->obstacles->sensor_signals[signal_nmbr] = distance; 

    if ((signal_nmbr == 1) || (signal_nmbr == 6)){
    theCtrlStruct->obstacles->sensor_angle[signal_nmbr] = 90 + angle;
    }
    else if ((signal_nmbr == 3) || (signal_nmbr == 8)){
    theCtrlStruct->obstacles->sensor_angle[signal_nmbr] = 90 - angle;
    }

}

void sensorwise_direct(int signal_nmbr, CtrlStruct *theCtrlStruct){

    double sensor_angle = theCtrlStruct->obstacles->sensor_angle[signal_nmbr]; 
    double signal = theCtrlStruct->obstacles->sensor_signals[signal_nmbr]; 
    double vision[360] = theCtrlStruct->obstacles->vision; 
    double obstacle_width = theCtrlStruct->obstacles->obstacle_width; 
    double robot_side = theCtrlStruct->obstacles->robot_side; 

    double seen_angle; 
    double seen_angle_degree; 
    double seen_angle_rounded; 
    double sensor_angle_rad; 

    if(signal != 0){
        seen_angle = atan(obstacle_width/(2*signal)); 
        seen_angle_degree = 180 * seen_angle/M_PI;
        seen_angle_rounded = round(seen_angle_degree); 

        for(int k=sensor_angle-seen_angle_rounded; k<sensor_angle+seen_angle_rounded; k++){
            if((vision[k] != 0) && (vision[k]>signal)) || (vision[k] == 0){
                sensor_angle_rad = M_PI*sensor_angle/180;
                vision[k]= signal + (robot_side/2) * (1/sin(sensor_angle_rad));
            }
        }
    }

    theCtrlStruct->obstacles->vision = vision; 
}

void locate_obstacles (CtrlStruct * theCtrlStruct){
    
    double vision[360] = theCtrlStruct->obstacles->vision; 
    double theta = theCtrlStruct->rob_pos->theta; 
    double x_360[360] = theCtrlStruct->obstacles->x_360;
    double y_360[360] = theCtrlStruct->obstacles->y_360; 
    double angle_of_sight = 0; 

    // adapting the orientation along with the one of the robot
    int orientation = round(theta*180/M_PI); 
    rotate(vision[0], orientation,vision[360]);

    // locating around the robot
    for (int k = 0; k < 360; k++)
    {
        if(vision[k]>0){
            angle_of_sight = k*M_PI/180; 
            x_360[k] = vision[k]*cos(angle_of_sight); 
            y_360[k] = vision[k]*sin(angle_of_sight); 
        }  
    }
    
    theCtrlStruct->obstacles->x_360 = x_360; 
    theCtrlStruct->obstacles->y_360 = y_360; 
}

void map_obstacles (CtrlStruct * theCtrlStruct){

    double x_360[360] = theCtrlStruct->obstacles->x_360; 
    double y_360[360] = theCtrlStruct->obstacles->y_360; 
    double map[200,300] = theCtrlStruct->obstacles->map; 

    double robot_x = theCtrlStruct->rob_pos->x; 
    double robot_y = theCtrlStruct->rob_pos->y;

    int rounded_robot_x = round(robot_x); 
    int rounded_robot_y = round(robot_y); 
    int map_obst_x = 0; 
    int map_obst_y = 0; 

    for (int k = 0; k < 360; k++)
    {
        map_obst_x = rounded_robot_x + round(x_360[k]); 
        map_obst_y = rounded_robot_y + round(y_360[k]); 

        if((map_obst_x>0)&&(map_obst_x<300)&&(map_obst_y>0)&&(map_obst_y<200)){
            map(map_obst_x, map_obst_y) = 1; 
        }

    }
    
    theCtrlStruct->obstacles->map = map; 

}