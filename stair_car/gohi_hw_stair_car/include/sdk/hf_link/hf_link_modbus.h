#ifndef HF_LINK_MODBUS_H
#define HF_LINK_MODBUS_H

#include "robot_abstract.h"
#include "hf_link_state_machine_modbus.h"
#include "robot_control.h"

//comand type
enum MotorModbusCommand{

    READ_MOT1_ERROR_STATE,    
    READ_MOT2_ERROR_STATE,
    READ_MOT3_ERROR_STATE,
    READ_MOT4_ERROR_STATE,
    READ_MOT5_ERROR_STATE,
    READ_MOT6_ERROR_STATE,
    
    READ_MOT1_SPEED,
    READ_MOT2_SPEED,
    READ_MOT3_SPEED,
    READ_MOT4_SPEED,
    READ_MOT5_SPEED,
    READ_MOT6_SPEED,
    
    READ_MOT1_REAL_POSITION,
    READ_MOT2_REAL_POSITION,
    READ_MOT3_REAL_POSITION,
    READ_MOT4_REAL_POSITION,
    READ_MOT5_REAL_POSITION,
    READ_MOT6_REAL_POSITION,
    
    READ_CAR1_MOTOR1_COMPLETE_STATE,
    READ_CAR1_MOTOR2_COMPLETE_STATE,
    READ_CAR2_MOTOR3_COMPLETE_STATE,
    READ_CAR2_MOTOR4_COMPLETE_STATE,
    READ_CAR3_MOTOR5_COMPLETE_STATE,
    READ_CAR3_MOTOR6_COMPLETE_STATE,
    READ_CAR2_MOTER3_RESET_STATE,
    
    SET_MOT1_BRAKE_STATE,
    SET_MOT2_BRAKE_STATE,
    SET_MOT3_BRAKE_STATE,
    SET_MOT4_BRAKE_STATE,
    SET_MOT5_BRAKE_STATE,
    SET_MOT6_BRAKE_STATE,
    
    SET_MOT3_FREE_BRAKE_STATE,
    SET_MOT3_NORMAL_BRAKE_STATE,

    SET_CAR1_LEFT_SPEED_CONTROL,
    SET_CAR1_RIGHT_SPEED_CONTROL,
    SET_CAR2_POSITION_CONTROL,
    SET_STAIR_CAR2_SPEED_CONTROL,
    SET_CAR2_SPEED_CONTROL,
    SET_CAR3_LEFT_SPEED_CONTROL,
    SET_CAR3_RIGHT_SPEED_CONTROL,
    SET_CAR5_SPEED_CONTROL,//兴颂驱动器

    SET_MOT3_RESET_TEST,
    
    READ_LAXIAN_POSITION,
    READ_EULER_ANGLE,
    SET_RELAY5_STATE,   //设置继电器的状态
    SET_RELAY6_STATE,   //设置继电器的状态
    SET_RELAY7_STATE,   //设置继电器的状态
    READ_RFID_REG_DATA,//读射频传感器
    SET_RFID_REG_DATA, //写射频传感器


    READ_INTERFACE_CAR1_SPEED_CONTROL,
    READ_INTERFACE_CAR2_SPEED_CONTROL,
    READ_INTERFACE_CAR3_POSITION_CONTROL,
    READ_INTERFACE_CAR4_SINGLE_SPEED_CONTROL,
    LAST_COMMAND_FLAG_};


enum ModbusCommandCode{
    READ_REG=0x03,
    READ_INPUT_REG=0x04,
    WRITE_REG=0x06,
    WRITE_MORE_REG=0x10,
    WRITE_RELAY_REG_OFF =0x00,
    WRITE_RELAY_REG_ON =0xff,
};


//控制继电器通断的命令
enum RelayControlCmd{
    RELAY5_ON  =0X20,
    RELAY6_ON  =0X40,
    STAIR_RELAY7_ON  =0X80,
    RELAY5_OFF  =0Xc0,
    RELAY6_OFF =0Xa0,
    STAIR_RELAY7_OFF  =0X60,

};


enum ModbusCommandRegAddr{
    READ_ERROR_STATE_ADDR=0x0033,
    READ_MOT_SPEED_ADDR=0x0022,
    READ_POSITION_COMPLETE_STATE_ADDR=0x0023,  
    READ_POSITION_RESET_STATE_ADDR=0x002d, 
    READ_MOT_POSITION_ADDR=0x0024,    
    SET_BRAKE_STATE_ADDR=0x0040,
    SET_MOT_SPEED_ADDR=0x0043,
    SET_MOT_POSITION_ADDR=0x0044,
    READ_LAXIAN_POSITION_ADDR=0x0000,
    READ_EULER_ANGLE_ADDR=0x003d,

    READ_RFID_REG_DATA_ADDR=0x03e8,  //读射频传感器寄存器首地址//1000
    SET_RFID_REG_DATA_ADDR=0x07D0,  //读射频传感器寄存器首地址//2000

    SET_CAR5_SPEED_ADDR=0x07D2,//兴颂驱动器
    
    READ_INTERFACE_CAR1_SPEED_CONTROL_ADDR=0x0001,              //1 x速速         2 y速度      3 Z速度
    READ_INTERFACE_CAR2_SPEED_CONTROL_ADDR=0x0004,              //1 x速速         2 y速度      3 Z速度
    READ_INTERFACE_CAR3_POSITION_CONTROL_ADDR=0x0007,           //1 速度          2 位置类型    3 位置
    READ_INTERFACE_CAR4_SINGLE_SPEED_CONTROL_ADDR=0x000A,        //1传送带1速度     2 传送带2速度 3 传送带3速度

    
    SET_MOT3_RESET_TEST_ADDR =0x00aa,
//first  modify
    SET_RELAY5_REG_ADDR   =0x05,
    SET_RELAY6_REG_ADDR   =0x06,
    SET_RELAY7_REG_ADDR   =0x07,
};


enum ModbusSlaveAddr{
  
    MOTOR1_ADDR=0x01,
    MOTOR2_ADDR=0x02,
    
    MOTOR3_ADDR=0x03,
    MOTOR4_ADDR=0x04,
    
    MOTOR5_ADDR=0x05,
    MOTOR6_ADDR=0x06,

    IMU_ADDR   =0x50,
    LAXIAN_ADDR=0x08,
    RFID_ADDR  =0x7F,
    PAD_INTERFACE_ADDR_CAR1  =0x70,    
    PAD_INTERFACE_ADDR_CAR2  =0x71,   
    PAD_INTERFACE_ADDR_CAR3  =0x72,    
    PAD_INTERFACE_ADDR_CAR4  =0x73,

    PAD_INTERFACE_ADDR_ODOM  =0x74,    
    PAD_INTERFACE_ADDR_ID    =0x75,
    PAD_INTERFACE_ADDR_LASER =0x76,
        
    SET_RELAY5_ADDR =0X05,
    SET_RELAY6_ADDR =0X06,
    SET_RELAY7_ADDR =0X07,
    };  //射频传感器设备地址//

#define BigLittleSwap16(A) ((((uint16)(A) & 0xff00) >> 8) | \ (((uint16)(A) & 0x00ff) << 8




class HFLink_Modbus : public StateMachineModbus
{
public:
    HFLink_Modbus(RobotAbstract* robot_  , unsigned char my_id_= 0x11 , unsigned char friend_id_= 0x01 , unsigned char port_num_ = 1) :
        StateMachineModbus(my_id_ , friend_id_ , port_num_)
    {
        hf_link_node_model = HF_LINK_NODE_MODEL ;
        //enable hflink ack , generally, master disable and slave enable
        //and slave also can disable to reduce communication burden
        hf_link_ack_en = 0;
        if(hf_link_node_model == 0) hf_link_ack_en = 1;

        robot=robot_;
        shaking_hands_state = 0;
        analysis_package_count  = 0;
        write_analysis_package_count=0;
        read_analysis_package_count=0;
        command_state_ = READ_MOT1_ERROR_STATE;

        stair_position_temp_temp =0;
        stair_position_complete_state_temp =0;
    }

public:  
    //only for master
    //the master can use masterSendCommand function to send data to slave
    //like SET_GLOBAL_SPEED , READ_ROBOT_SYSTEM_INFO, READ_ROBOT_SPEED...
    unsigned char masterSendCommand(const MotorModbusCommand command_state);
    unsigned char masterSendCommand(const MotorModbusCommand command_state,int relay_on_or_off);
    inline unsigned char getReceiveRenewFlag(const MotorModbusCommand command_state) const
    {
        return receive_package_renew[command_state];
    }
    inline Robot_Control* getRobotControl()
    {
        return &robot_control;
    }
    // float get_d_past_angel(void){    //recording d angle for robot coordinate calculation
    //     float temp=robot->ask_motor_control_data.d_past_angle;
    //     robot->ask_motor_control_data.d_past_angle=0;
    //     return temp;
    // }    

public: 
    //only for slave
    //command updata flag , the robot need to traverse These flag to decide update his own behavior
    unsigned char receive_package_renew[LAST_COMMAND_FLAG_];



public: 
     float stair_position_temp_temp;
     unsigned short stair_position_complete_state_temp;
     unsigned char stair_reset_SQ_state_temp;
public:  
    //common
    unsigned char byteAnalysisCall(const unsigned char rx_byte);  
    unsigned char byteAnalysisCall_R(const unsigned char rx_byte);  
     
    //unsigned char writeByteAnalysisCall(const unsigned char rx_byte);
    //unsigned char readByteAnalysisCall(const unsigned char rx_byte);
    
    inline void enable_ack(void){if(hf_link_ack_en != 1) hf_link_ack_en=1;}
    inline void disable_ack(void){hf_link_ack_en=0;}

private:
    unsigned char hf_link_node_model;      // 0 slave , 1 master
    unsigned char hf_link_ack_en;          //enable hflink ack
    unsigned char shaking_hands_state;     //1 Success   0 Failed
    float analysis_package_count;
    float write_analysis_package_count;
    float read_analysis_package_count;
    float control_quality ;    
    float past_total_angle[4];
    float d_past_angle[4];
    float measure_angle_speed[4] ;
    
    

    RobotAbstract* robot;      //robot abstract pointer to hflink
    Robot_Control robot_control;
    MotorModbusCommand    command_state_;

    unsigned char packageAnalysis(void);
    unsigned char readCommandAnalysis(const MotorModbusCommand command_state , unsigned char* p , const unsigned short int len);
    unsigned char readCommandAnalysis(const MotorModbusCommand command_state ,short int* p ,  unsigned short int len);
    unsigned char readCommandAnalysis(const MotorModbusCommand command_state , int* p ,  unsigned short int len);

    unsigned char setCommandAnalysis(const MotorModbusCommand command_state , float* p , const unsigned short int len);
    // void sendStruct(const Command command_state , unsigned char* p , const unsigned short int len);
    void sendStruct(const ModbusSlaveAddr slave_addr,const ModbusCommandCode command_Code, const ModbusCommandRegAddr command_reg_addr , unsigned char* p ,  unsigned short int len);
    void sendStruct(const ModbusSlaveAddr slave_addr,const ModbusCommandCode command_Code, const ModbusCommandRegAddr command_reg_addr , float* p ,  unsigned short int len);
    void sendStructToPAD(const ModbusSlaveAddr slave_addr,const ModbusCommandCode command_Code, const ModbusCommandRegAddr command_reg_addr , unsigned char* p ,  unsigned short int len);

};
#endif  // #ifndef HF_LINK_H

