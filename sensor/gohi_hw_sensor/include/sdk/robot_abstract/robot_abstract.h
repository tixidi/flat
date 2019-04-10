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


#include "laxian_parameters.h"

#include "interface.h"


#include "imu_info.h"
#include "system_info.h"
#include "string.h"

static const unsigned short int SEND_TO_PAD_BUFER_SIZE = 1024;

class RobotParameters
{
public:
    RobotParameters(){
        degree_to_radian = 0.017453f;
        radian_to_degree = 57.2958f;

    }

public:

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



        /************************************IMU Sensors******************************************/
        memset(&gyro_acc , 0 , sizeof(gyro_acc));
        memset(&magnetic_fusion , 0 , sizeof(magnetic_fusion));
        memset(&euler_angle , 0 , sizeof(euler_angle));
        

 
    /************************************LAXIAN LENGTH********************************************/
        memset(&laxian_length , 0 , sizeof(laxian_length));
    /************************************RFID LENGTH********************************************/
        memset(&rfid_write_data , 0 , sizeof(rfid_write_data));
        memset(&rfid_read_data , 0 , sizeof(rfid_read_data));
    /************************************INTERFACE********************************************/
        memset(&car1_speed_config , 0 , sizeof(car1_speed_config));
        memset(&car2_speed_config , 0 , sizeof(car2_speed_config));
        memset(&car3_position_config , 0 , sizeof(car3_position_config));
        memset(&car4_single_speed_config , 0 , sizeof(car4_single_speed_config));
        memset(&car_global_position_config , 0 , sizeof(car_global_position_config));
        memset(&laser_range_config , 0 , sizeof(laser_range_config));

    /************************************INTERFACE********************************************/  


        memset(&laser_scan_data,0,sizeof(laser_scan_data));
        memset(&laser_scan_num,0,sizeof(laser_scan_num));
        memset(&laser_scan_range,270,sizeof(laser_scan_range));  //默认270个值
        memset(&laser_scan_resolution,1,sizeof(laser_scan_resolution)); //默认分辨率为1度
        
        

        

    }

    /************************************system info*********************************************/
    SystemInfo system_info;   //(meter,meter,factor(0~1))

    unsigned char receive_package_flag;
    /************************************robot parameters*********************************************/
    //unit  distances : metres
    //angle： radian    void chassisDatatUpdate(void);
    RobotParameters para;




    /************************************IMU sensors********************************************/
    IMUSensorData gyro_acc , magnetic_fusion; //(pitch,roll,yaw)(radian,radian,radian)
    IMUEulerData  euler_angle;
    GPSData gps_data;
    /************************************AISIKONG MOTORS********************************************/

    /************************************LAXIAN LENGTH********************************************/

    LaXianLengthData laxian_length;
    /************************************RFID数据********************************************/
    RfidWriteRegData  rfid_write_data;
    RfidReadRegData   rfid_read_data;

    SET_MOT3_SPEED   MOT3_SPEED;
    /************************************温度传感器********************************************/
    ThermometerReadRegData Temperature_Data;


    /************************************INTERFACE数据********************************************/
    CarSpeedConfig  car1_speed_config,car2_speed_config;  
    CarPositionConfig car3_position_config;
    CarSingleSpeedConfig car4_single_speed_config;
    CarGLobalPositionConfig car_global_position_config;
    LaserRangeConfig   laser_range_config;

    /*************************************************************************************/

    short int laser_scan_data[SEND_TO_PAD_BUFER_SIZE];
    unsigned short int laser_scan_num;
    unsigned short int laser_scan_range;
    unsigned short int laser_scan_resolution;
    
    




};


#endif // ROBOT_ABSTRACT_H

