#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "stdio.h"
#include <iostream>
#include <string>
#include <math.h> 

#define motor_num 2
#define HF_LINK_NODE_MODEL  1    //1:master(PC)  0 :slave(MCU)



#define wheel_radius  0.0510//0.0625
#define body_radius   0.2825//0.565
#define degree_to_radian  0.017453f
#define radian_to_degree  57.2958f


class Robot_Control
{
public:
    Robot_Control()
    {

    }

    void robotSpeedSet(const float* expect_robot_speed , float* expect_motor_angle_speed);    
    void robotToMotorTF(const float* robot , float* motor );
    void getRobotSpeed(const float* measure_motor_angle_speed , float* measure_robot_speed);
    void motorToRobotTF(const float* motor , float* robot)  ; 
    void getGlobalCoordinate(const float* d_motor_len  , float* measure_global_coordinate);
    void motorToGlobalTF(const float* motor , float* global ,float R_theta);        




};

#endif // HF_LINK_STATE_MACHINE_H
