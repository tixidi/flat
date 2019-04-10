/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: robot_abstract.h
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
*
* Description:
***********************************************************************************************************************/

#ifndef ROBOT_ABSTRACT_H
#define ROBOT_ABSTRACT_H

#include "aisikong_parameters.h"

#include "chassis_parameters.h"



#include "imu_info.h"
#include "system_info.h"
#include "string.h"

class RobotParameters
{
public:
    RobotParameters(){
        degree_to_radian = 0.017453f;
        radian_to_degree = 57.2958f;
        // memset(&motor_para , 0 , sizeof(motor_para));
        memset(&chassis_para , 0 , sizeof(chassis_para));
        // memset(&head_para , 0 , sizeof(head_para));
        // memset(&arm_para , 0 , sizeof(arm_para));
    }

public:
    // MotorParameters motor_para;
    ChassisParameters chassis_para;
    // HeadParameters head_para;
    // ArmParameters arm_para;
    float degree_to_radian ,  radian_to_degree;
};

class RobotAbstract
{
public:
    RobotAbstract()
    {
        para = RobotParameters();

        /************************************system info*******************************************/
        memset(&system_info , 0 , sizeof(system_info));

        /************************************motor info*******************************************/


        /************************************Chassis**********************************************/
        memset(&expect_motor_speed , 0 , sizeof(expect_motor_speed));
        memset(&measure_motor_speed , 0 , sizeof(measure_motor_speed));
        memset(&expect_robot_speed , 0 , sizeof(expect_robot_speed));
        memset(&measure_robot_speed , 0 , sizeof(measure_robot_speed));
        memset(&expect_global_speed , 0 , sizeof(expect_global_speed));
        memset(&measure_global_speed , 0 , sizeof(measure_global_speed));
        memset(&measure_motor_mileage , 0 , sizeof(measure_motor_mileage));
        memset(&measure_global_coordinate , 0 , sizeof(measure_global_coordinate));
        memset(&measure_robot_coordinate , 0 , sizeof(measure_robot_coordinate));

        /************************************arm*************************************************/
        // memset(&expect_arm_state , 0 , sizeof(expect_arm_state));
        // memset(&measure_arm_state , 0 , sizeof(measure_arm_state));

        // /************************************head************************************************/
        // memset(&expect_head_state , 0 , sizeof(expect_head_state));
        // memset(&measure_head_state , 0 , sizeof(measure_head_state));
        /************************************IMU Sensors******************************************/
        memset(&gyro_acc , 0 , sizeof(gyro_acc));
        memset(&magnetic_fusion , 0 , sizeof(magnetic_fusion));
        memset(&euler_angle , 0 , sizeof(euler_angle));
        

        /************************************AISIKONG MOTORS********************************************/
        memset(&motor_error_state , 0 , sizeof(motor_error_state));
        memset(&motor_pos_comp_state , 0 , sizeof(motor_pos_comp_state));
        memset(&ask_measure_motor_speed , 0 , sizeof(ask_measure_motor_speed));
        memset(&ask_measure_motor_position , 0 , sizeof(ask_measure_motor_position));
        memset(&ask_measure_motor_position_last , 0 , sizeof(ask_measure_motor_position_last));
        memset(&ask_measure_motor_position_dif , 0 , sizeof(ask_measure_motor_position_dif));    

        
        memset(&ask_expect_motor_speed , 0 , sizeof(ask_expect_motor_speed));
        memset(&ask_brake_config , 0 , sizeof(ask_brake_config));
        memset(&ask_position_config , 0 , sizeof(ask_position_config));
        memset(&ask_position_set , 0 , sizeof(ask_position_set));
        

        memset(&ask_motor_d_past_angle , 0 , sizeof(ask_motor_d_past_angle));
    // /************************************LAXIAN LENGTH********************************************/
    //     memset(&laxian_length , 0 , sizeof(laxian_length));
    // /************************************RFID LENGTH********************************************/
    //     memset(&rfid_write_data , 0 , sizeof(rfid_write_data));
    //     memset(&rfid_read_data , 0 , sizeof(rfid_read_data));
    /************************************INTERFACE********************************************/
        // memset(&car1_speed_config , 0 , sizeof(car1_speed_config));
        // memset(&car2_speed_config , 0 , sizeof(car2_speed_config));
        // memset(&car3_position_config , 0 , sizeof(car3_position_config));
        // memset(&car4_single_speed_config , 0 , sizeof(car4_single_speed_config));

    /************************************INTERFACE********************************************/  
        // memset(&MOT3_SPEED , 0 , sizeof(MOT3_SPEED));

        // memset(&XS_ABMotor_Control , 0 , sizeof(XS_ABMotor_Control));

     /*******************************************************/   
     
        memset(&dilong_speed , 0 , sizeof(dilong_speed));
        memset(&stair_positionPhaseChange , 0 , sizeof(stair_positionPhaseChange));
        memset(&stair_type , 0 , sizeof(stair_type));
        memset(&stair_position , 0 , sizeof(stair_position));
        memset(&stair_stop_flag , 0 , sizeof(stair_stop_flag));
        
        
       
    }

    /************************************system info*********************************************/
    SystemInfo system_info;   //(meter,meter,factor(0~1))

    /************************************robot parameters*********************************************/
    //unit  distances : metres
    //angle： radian    void chassisDatatUpdate(void);
    RobotParameters para;

    /************************************chassis************************************************/
    ChassisDOFVector  expect_motor_speed;   //(x1,x2,x3)(radian/s,radian/s,radian/s)
    ChassisDOFVector  measure_motor_speed;
    ChassisCoord   expect_robot_speed;   //(x,y,w)(meter/s,meter/s,radian/s) reference system:robot
    ChassisCoord   measure_robot_speed;
    ChassisCoord   expect_global_speed;  //(x,y,w)(meter/s,meter/s,radian/s) reference system:global such as /map /odmo ;
    ChassisCoord   measure_global_speed;
    ChassisDOFVector  measure_motor_mileage; //(x1,x2,x3)(radian,radian,radian)
    ChassisCoord   measure_global_coordinate;  //(x,y,w)(meter,meter,radian)
    ChassisCoord   measure_robot_coordinate;

    /************************************arm***************************************************/
    // ArmDOFVector expect_arm_state;
    // ArmDOFVector measure_arm_state;

    // /************************************head**************************************************/
    // HeadPose   expect_head_state;    //(pitch,roll,yaw)(radian,radian,radian)
    // HeadPose   measure_head_state;

    /************************************IMU sensors********************************************/
    IMUSensorData gyro_acc , magnetic_fusion; //(pitch,roll,yaw)(radian,radian,radian)
    IMUEulerData  euler_angle;
    GPSData gps_data;
    /************************************AISIKONG MOTORS********************************************/
    AiSiKongMotorErrorState       motor_error_state;
    AiSiKongPositionCompleteState motor_pos_comp_state;
    AiSiKongMotorDOFVector        ask_measure_motor_speed;
    AiSiKongMotorRTPosition       ask_measure_motor_position;
    AiSiKongMotorRTPositionLast   ask_measure_motor_position_last;
    AiSiKongMotorRTPositionDif    ask_measure_motor_position_dif;
    AiSiKongMotorDOFVector        ask_expect_motor_speed;
    AiSiKongBrakeConfig           ask_brake_config;
    AiSiKongPositionConfig        ask_position_config;
    AiSiKongPositionSet           ask_position_set;

    AiSiKongMotorDPastAngle       ask_motor_d_past_angle;
    /************************************LAXIAN LENGTH********************************************/

    // LaXianLengthData laxian_length;
    /************************************RFID数据********************************************/
    // RfidWriteRegData  rfid_write_data;
    // RfidReadRegData   rfid_read_data;

    // SET_MOT3_SPEED   MOT3_SPEED;

     /************************************兴颂驱动器********************************************/
    // XingSongMotorControl  XS_ABMotor_Control; //兴颂驱动器AB电机控制

    /************************************INTERFACE数据********************************************/
    // CarSpeedConfig  car1_speed_config,car2_speed_config;  
    // CarPositionConfig car3_position_config;
    // CarSingleSpeedConfig car4_single_speed_config; 

    float dilong_speed;
    float stair_positionPhaseChange;
    float  stair_type;
    float stair_position;
    unsigned char stair_stop_flag;

    

};


#endif // ROBOT_ABSTRACT_H

