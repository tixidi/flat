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


#include "interface.h"

#include "laxian_parameters.h"

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


        /************************************Chassis**********************************************/

        /************************************IMU Sensors******************************************/
        memset(&gyro_acc , 0 , sizeof(gyro_acc));
        memset(&magnetic_fusion , 0 , sizeof(magnetic_fusion));
        memset(&euler_angle , 0 , sizeof(euler_angle));
        

        /************************************AISIKONG MOTORS********************************************/


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
        memset(&MOT3_SPEED , 0 , sizeof(MOT3_SPEED));


        
        memset(&Temperature_Data , 0 , sizeof(Temperature_Data));
        
        memset(&laser_scan_data,0,sizeof(laser_scan_data));
        memset(&laser_scan_num,0,sizeof(laser_scan_num));
        memset(&laser_scan_range,270,sizeof(laser_scan_range));  //默认270个值
        memset(&laser_scan_resolution,1,sizeof(laser_scan_resolution)); //默认分辨率为1度

        memset(&ack_to_pad_sensor_data_type,0,sizeof(ack_to_pad_sensor_data_type)); //默认分辨率为1度
        memset(&ack_to_pad_laser_data_type,0,sizeof(ack_to_pad_laser_data_type)); //默认分辨率为1度
        
        memset(&bms_battey_,0,sizeof(BmsBatteryDate)); //默认分辨率为1度
        
         memset(&moter_error_state_,0,sizeof(MoterErrorState)); //默认分辨率为1度
        memset(&moter_speed_state_,0,sizeof(MoterSpeedState)); //默认分辨率为1度

    }
    MoterSpeedState  moter_speed_state_;
    MoterErrorState  moter_error_state_;
    /************************************system info*********************************************/
    SystemInfo system_info;   //(meter,meter,factor(0~1))

    unsigned char receive_package_flag;
    
    /************************************robot parameters*********************************************/
    //unit  distances : metres
    //angle： radian    void chassisDatatUpdate(void);
    RobotParameters para;

    /************************************chassis************************************************/

    /************************************arm***************************************************/


    /************************************IMU sensors********************************************/
    IMUSensorData gyro_acc , magnetic_fusion; //(pitch,roll,yaw)(radian,radian,radian)
    IMUEulerData  euler_angle;
    GPSData gps_data;

    /************************************LAXIAN LENGTH********************************************/


    LaXianLengthData laxian_length;
    /************************************RFID数据********************************************/
    RfidWriteRegData  rfid_write_data;
    RfidReadRegData   rfid_read_data;

    SET_MOT3_SPEED   MOT3_SPEED;
    /************************************温度传感器********************************************/
    ThermometerReadRegData Temperature_Data;
     /************************************兴颂驱动器********************************************/


    /************************************INTERFACE数据********************************************/
    CarSpeedConfig  car1_speed_config,car2_speed_config;  
    CarPositionConfig car3_position_config;
    CarSingleSpeedConfig car4_single_speed_config;
    CarGLobalPositionConfig car_global_position_config;
    LaserRangeConfig   laser_range_config;


    AckToPadDataType  ack_to_pad_sensor_data_type;
    AckToPadDataType  ack_to_pad_laser_data_type;
    /******************************电池参数*******************************************************/
    BmsBatteryDate bms_battey_;
    /*************************************************************************************/
    short int laser_scan_data[SEND_TO_PAD_BUFER_SIZE];
    unsigned short int laser_scan_num;
    unsigned short int laser_scan_range;
    unsigned short int laser_scan_resolution;
    


};


#endif // ROBOT_ABSTRACT_H

