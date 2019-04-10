#include "hf_link_modbus.h"
#include "stdio.h"
#include <string.h>
#include <iostream>


unsigned char HFLink_Modbus::byteAnalysisCall(const unsigned char rx_byte)
{
          
    if( receiveStates(rx_byte) )
    {
        //receive a new message
        unsigned char package_update = packageAnalysis();
        if(package_update == 1) 
        {
            write_analysis_package_count++;        
            // std::cerr << "write Out" <<write_analysis_package_count<<std::endl;
        }
        return package_update;
    }
    return 0;
}

unsigned char HFLink_Modbus::byteAnalysisCall_R(const unsigned char rx_byte)
{

    if( receiveStates_R(rx_byte) )
    {      
        //receive a new message
        unsigned char package_update = packageAnalysis();
        if(package_update == 1)
        {
            read_analysis_package_count++;
            // std::cerr << "read Out" <<read_analysis_package_count<<std::endl;
        }

        return package_update;
    }
    return 0;
}

/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:    stm32f4+fpu(1 us)
*
* History:
***********************************************************************************************************************/
unsigned char HFLink_Modbus::packageAnalysis(void)
{
    short int temp_mea_servo1_speed;        
    short int temp_mea_servo2_speed ;

    short int temp_euler_angle[3] ; 
   
    int temp_mea_motor1_position;
    int temp_mea_motor2_position;
 
    float per_meter_odometry = 0;
    float meter_flag = 0;

    static int read_count1=0;
    static int read_count2=0;

    float  per_circle_position =(360/120)*8*30;
    float   pid_t  =0.02*6;


    if(robot == NULL){
        printf(" error , the robot is NULL  \n");
        return 0;
    }

    if(rx_message.slave_addr==MOTOR1_ADDR)
    {
        if(rx_message.slave_reg_addr==READ_ERROR_STATE_ADDR)                     command_state_=READ_MOT1_ERROR_STATE;        
        else if(rx_message.slave_reg_addr==READ_MOT_SPEED_ADDR)                  command_state_=READ_MOT1_SPEED;
        else if(rx_message.slave_reg_addr==READ_POSITION_COMPLETE_STATE_ADDR)    command_state_=READ_CAR1_MOTOR1_COMPLETE_STATE;
        else if(rx_message.slave_reg_addr==READ_MOT_POSITION_ADDR)               command_state_=READ_MOT1_REAL_POSITION;
        else if(rx_message.slave_reg_addr==SET_BRAKE_STATE_ADDR)                 command_state_=SET_MOT1_BRAKE_STATE;
        else if(rx_message.slave_reg_addr==SET_MOT_SPEED_ADDR)                   command_state_=SET_CAR1_LEFT_SPEED_CONTROL;
        else                                                                     command_state_=LAST_COMMAND_FLAG_;
    }
    else if(rx_message.slave_addr==MOTOR2_ADDR)
    {
        if(rx_message.slave_reg_addr==READ_ERROR_STATE_ADDR)                     command_state_=READ_MOT2_ERROR_STATE;        
        else if(rx_message.slave_reg_addr==READ_MOT_SPEED_ADDR)                  command_state_=READ_MOT2_SPEED;       
        else if(rx_message.slave_reg_addr==READ_POSITION_COMPLETE_STATE_ADDR)    command_state_=READ_CAR1_MOTOR2_COMPLETE_STATE;
        else if(rx_message.slave_reg_addr==READ_MOT_POSITION_ADDR)               command_state_=READ_MOT2_REAL_POSITION; 
        else if(rx_message.slave_reg_addr==SET_BRAKE_STATE_ADDR)                 command_state_=SET_MOT2_BRAKE_STATE;
        else if(rx_message.slave_reg_addr==SET_MOT_SPEED_ADDR)                   command_state_=SET_CAR1_RIGHT_SPEED_CONTROL;
        else                                                                     command_state_=LAST_COMMAND_FLAG_;
    }   
    else if(rx_message.slave_addr==IMU_ADDR)
    {       
        if(rx_message.slave_reg_addr==READ_EULER_ANGLE_ADDR)                     command_state_=READ_EULER_ANGLE;
    }


    unsigned char analysis_state =0;

    switch (command_state_)
    {
    case READ_MOT1_ERROR_STATE :
        // if(receive_package_renew[(unsigned char)command_state_]==0)     
        {
            analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->motor_error_state.error1 , sizeof(robot->motor_error_state.error1));
            // std::cerr <<"READ_MOT1_ERROR_STATE     eeeeeeeeeeeeeeeeeeeeeeeeeeee11111111111111                  "  <<read_count1++<<"   "<<(unsigned int )robot->motor_error_state.error1 <<std::endl;    
        }
        break;
    case READ_MOT2_ERROR_STATE :
        // if(receive_package_renew[(unsigned char)command_state_]==0)  
        {
            analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->motor_error_state.error2 , sizeof(robot->motor_error_state.error2));
            // std::cerr <<"READ_MOT2_ERROR_STATE     eeeeeeeeeeeeeeeeeeeeeeeeeeee22222222222222                 "  <<read_count2++<<"   "<<(unsigned int )robot->motor_error_state.error2 <<std::endl;  
        }
        break;

    case READ_MOT1_REAL_POSITION:
        
        analysis_state=readCommandAnalysis(command_state_ , (int* )&temp_mea_motor1_position , sizeof(robot->ask_measure_motor_position.position1));
        robot->ask_measure_motor_position.position1=(float)temp_mea_motor1_position;
  
        robot->ask_measure_motor_position_dif.position1=robot->ask_measure_motor_position.position1-robot->ask_measure_motor_position_last.position1;
        robot->ask_measure_motor_position_last.position1=robot->ask_measure_motor_position.position1;
        //recording total angle for robot coordinate calculation
        d_past_angle[0] = (robot->ask_measure_motor_position_dif.position1/per_circle_position)*360;
        past_total_angle[0]+= ( robot->ask_measure_motor_position_dif.position1/per_circle_position)*360;

        //calc motor speed  degree/s
        
        robot->ask_measure_motor_speed.servo1=  (robot->ask_measure_motor_position_dif.position1) * 360 / ( per_circle_position*pid_t )*degree_to_radian ;
    	// std::cerr <<"电机1速度: " <<robot->ask_measure_motor_speed.servo1<<" rad/s" <<std::endl; //读电机1位置
        
        break;

    case READ_MOT2_REAL_POSITION :  
        // d_past_angle[1] =0;            
        analysis_state=readCommandAnalysis(command_state_ , (int* )&temp_mea_motor2_position , sizeof(robot->ask_measure_motor_position.position2));
        robot->ask_measure_motor_position.position2=(float)temp_mea_motor2_position;

        robot->ask_measure_motor_position_dif.position2=robot->ask_measure_motor_position.position2-robot->ask_measure_motor_position_last.position2;
        robot->ask_measure_motor_position_last.position2=robot->ask_measure_motor_position.position2;
        //recording total angle for robot coordinate calculation
        d_past_angle[1] = (robot->ask_measure_motor_position_dif.position2/per_circle_position)*360;
        past_total_angle[1]+= ( robot->ask_measure_motor_position_dif.position2/per_circle_position)*360;

        //calc motor speed  degree/s
   
        robot->ask_measure_motor_speed.servo2=   robot->ask_measure_motor_position_dif.position2 * 360 / ( per_circle_position*pid_t )*degree_to_radian ;
        // std::cerr <<"电机2速度: " <<(robot->ask_measure_motor_speed.servo2)<<" rad/s" <<std::endl;
        
        break;
        


    case SET_MOT1_BRAKE_STATE :
        robot->ask_brake_config.brake1  =1;
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->ask_brake_config.brake1 , sizeof(robot->ask_brake_config.brake1));
        break;   
    case SET_MOT2_BRAKE_STATE :
        robot->ask_brake_config.brake2 =1;
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->ask_brake_config.brake2 , sizeof(robot->ask_brake_config.brake2));
        break;  
 

    case SET_CAR1_LEFT_SPEED_CONTROL :
        analysis_state=setCommandAnalysis(command_state_ , (float* )&robot->ask_expect_motor_speed.servo1 , sizeof(robot->ask_expect_motor_speed.servo1) );
        break;
    case SET_CAR1_RIGHT_SPEED_CONTROL :
        analysis_state=setCommandAnalysis(command_state_ , (float* )&robot->ask_expect_motor_speed.servo2 , sizeof(robot->ask_expect_motor_speed.servo2) );
        break;   
        // robot->expect_robot_speed.x=0.1;
        // robot->expect_robot_speed.y=0.0;
        // robot->expect_robot_speed.z=0.0;

    case READ_EULER_ANGLE :    
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->euler_angle , sizeof(robot->euler_angle) );   
        std::cerr <<"measure pitch  " <<robot->euler_angle.pitch*180.0/32768  <<std::endl;  
        std::cerr <<"measure roll  " <<robot->euler_angle.roll *180.0/32768<<std::endl;   
        std::cerr <<"measure yaw  " <<robot->euler_angle.yaw*180.0/32768<<std::endl;      
        break;

    default :
        analysis_state = 0;
        break;

    }
    rx_message.slave_addr=0;
    rx_message.command_id=0;
    rx_message.slave_reg_addr=0;
    memset(&rx_message.data , 0 , sizeof(rx_message.data));

    tx_message.slave_addr=0;    //clear flag
    tx_message.command_id=0;
    tx_message.slave_reg_addr=0;
    memset(&tx_message.data , 0 , sizeof(tx_message.data));
    
    return analysis_state;
}

/***********************************************************************************************************************
* Function:    void HFLink::masterSendCommand(Command command)
*
* Scope:       public
*
* Description: send a command or data to the friend_id
*              this function is olny belongs to master
*
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/

unsigned char HFLink_Modbus::masterSendCommand(const MotorModbusCommand command_state)
{
   short int temp_ask_servo1_speed ;
   short int temp_ask_servo2_speed ;
   
   void *single_command;




   receive_package_renew[(unsigned char)command_state] = 0 ;

    switch (command_state)
    {
    case READ_MOT1_ERROR_STATE :
        sendStruct(MOTOR1_ADDR , READ_REG,READ_ERROR_STATE_ADDR,(unsigned char *)single_command , 0);
        break;
    case READ_MOT2_ERROR_STATE :
        sendStruct(MOTOR2_ADDR , READ_REG,READ_ERROR_STATE_ADDR,(unsigned char *)single_command , 0);
        break;   
   
    case READ_MOT1_REAL_POSITION :
        sendStruct(MOTOR1_ADDR , READ_REG,READ_MOT_POSITION_ADDR,(unsigned char *)single_command , 0);
        break;
    case READ_MOT2_REAL_POSITION :
        sendStruct(MOTOR2_ADDR , READ_REG,READ_MOT_POSITION_ADDR,(unsigned char *)single_command , 0);
        break;

    case SET_MOT1_BRAKE_STATE :
        robot->ask_brake_config.brake1=2;
        sendStruct(MOTOR1_ADDR , WRITE_REG,SET_BRAKE_STATE_ADDR, (unsigned char *)&robot->ask_brake_config.brake1 , sizeof(robot->ask_brake_config.brake1));  
        break;
    case SET_MOT2_BRAKE_STATE :
        robot->ask_brake_config.brake2=2;
        sendStruct(MOTOR2_ADDR , WRITE_REG,SET_BRAKE_STATE_ADDR, (unsigned char *)&robot->ask_brake_config.brake2 , sizeof(robot->ask_brake_config.brake2));     
        break;


    case SET_CAR1_LEFT_SPEED_CONTROL :
        // robot->ask_expect_motor_speed.servo1=6.28;// rad/s
        // std::cerr <<"command5_____" <<robot->ask_expect_motor_speed.servo1 <<std::endl;//setup 
        robot->ask_expect_motor_speed.servo1=(robot->ask_expect_motor_speed.servo1)*60/2/3.14; //rpm
        robot->ask_expect_motor_speed.servo1=robot->ask_expect_motor_speed.servo1* (8*30)/0.1/20;  

        temp_ask_servo1_speed     =(short int )robot->ask_expect_motor_speed.servo1;
        sendStruct(MOTOR1_ADDR , WRITE_REG,SET_MOT_SPEED_ADDR, (unsigned char *)&temp_ask_servo1_speed, sizeof(temp_ask_servo1_speed) );      
        break;
    case SET_CAR1_RIGHT_SPEED_CONTROL :

        // robot->ask_expect_motor_speed.servo2=-6.28;// rad/s
        // std::cerr <<"command6______" <<robot->ask_expect_motor_speed.servo2 <<std::endl;
        robot->ask_expect_motor_speed.servo2=(robot->ask_expect_motor_speed.servo2)*60/2/3.14;
        robot->ask_expect_motor_speed.servo2=robot->ask_expect_motor_speed.servo2* (8*30)/0.1/20;
        temp_ask_servo2_speed     =(short int )robot->ask_expect_motor_speed.servo2;        
        sendStruct(MOTOR2_ADDR , WRITE_REG,SET_MOT_SPEED_ADDR, (unsigned char *)&temp_ask_servo2_speed, sizeof(temp_ask_servo2_speed) );
        break;                     
    case READ_EULER_ANGLE:
        sendStruct(IMU_ADDR , READ_REG,READ_EULER_ANGLE_ADDR,(unsigned char *)single_command , 0);           
        break;
   default :  
        break;
    } 
    return 1;
}
/***********************************************************************************************************************

* Description:
*
* Arguments:
*
* Return:Request16
***************************************************************** ******************************************************/
unsigned char HFLink_Modbus::readCommandAnalysis(const MotorModbusCommand  command_state , unsigned char* p ,  unsigned short int len)
{
    if (hf_link_node_model == 1)
    { // master   , means the slave feedback a package to master , and the master save this package

        memcpy(p , &rx_message.data[2] , len);
        receive_package_renew[(unsigned char)command_state] = 1 ;
    }

    return 1;
}
unsigned char HFLink_Modbus::readCommandAnalysis(const MotorModbusCommand command_state ,short int* p ,  unsigned short int len)
{
    if (hf_link_node_model == 1)
    { // master   , means the slave feedback a package to master , and the master save this package

        memcpy(p , &rx_message.data[0] , len);     

        receive_package_renew[(unsigned char)command_state] = 1 ;
    }

    return 1;
}
unsigned char HFLink_Modbus::readCommandAnalysis(const MotorModbusCommand command_state , int* p ,  unsigned short int len)
{
    if (hf_link_node_model == 1)
    { 
        memcpy(p , &rx_message.data[0] , len);     
        receive_package_renew[(unsigned char)command_state] = 1 ;
    }
    return 1;
}


unsigned char HFLink_Modbus::setCommandAnalysis(const MotorModbusCommand command_state , float* p ,  unsigned short int len)
{

    if (hf_link_node_model == 1)
    { 
        printf("I'm master , received a ack ");
        receive_package_renew[(unsigned char)command_state] = 1 ;
    }
    
    return 1;
}
/***********************************************************************************************************************
* Function:    void HFLink::sendStruct(const Command command_state , unsigned char* p , const unsigned short int len)
*
* Scope:       private
*
* Description:spend time is  0
* len =0       send a Single command to the friendspend time is  0
*              if i am slave , it can be  feed back a ack to master or requspend time is  0est instructions  like SHAKING_HANDS
*              if i am master , it can be s             ome request instrucspend time is  0tions like READ_ROBOT_SYSTEM_INFO READ_xxx
*
*
* len>0 :      send a Struct command to the friend hf_link nodeif
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/

void HFLink_Modbus::sendStruct(const ModbusSlaveAddr slave_addr,const ModbusCommandCode command_Code, const ModbusCommandRegAddr command_reg_addr , unsigned char* p ,  unsigned short int len)
{
    tx_message.slave_addr =  (unsigned char)slave_addr;
    tx_message.command_id =  (unsigned char)command_Code;
    tx_message.slave_reg_addr=(unsigned short int )command_reg_addr;
    //tx_message.data[0]=len;
    if(len > 0)
    {
        memcpy(&tx_message.data[0] , p , len);
    }
    switch (command_Code)
    {
    case READ_REG :
        Request03(&tx_message);

        break;

    case READ_INPUT_REG :
        Request04(&tx_message);
        break;


    case WRITE_REG :
        Request06(&tx_message);

        break;
        
    case WRITE_MORE_REG :
        Request16(&tx_message,len);
        break;

    default :
        break;
    }
}

void HFLink_Modbus::datatUpdate(void)
{

        // robot->expect_robot_speed.x=0.2;
        // robot->expect_robot_speed.y=0.0;
        // robot->expect_robot_speed.z=0.0;

        robot_control.robotSpeedSet((float* )&robot->expect_robot_speed , (float * )&robot->ask_expect_motor_speed);   //平仓机只修改这里，expect_motor_speed需要转换为相同的单位     
        robot_control.getRobotSpeed((float* )&robot->ask_measure_motor_speed , (float* )&robot->measure_robot_speed);

        // std::cerr <<"measure m1 speed  " <<robot->ask_measure_motor_speed.servo1 <<std::endl;
        // std::cerr <<"measure m2 speed  " <<robot->ask_measure_motor_speed.servo2 <<std::endl;
        // std::cerr <<"measure m3 speed  " <<robot->ask_measure_motor_speed.servo3<<std::endl;
        // std::cerr <<"X speed  " <<robot->measure_robot_speed.x <<std::endl;
        // std::cerr <<"Y speed  " <<robot->measure_robot_speed.y<<std::endl;

        robot->measure_motor_mileage.servo1 = past_total_angle[0] ;//degree
        robot->measure_motor_mileage.servo2 = past_total_angle[1] ;//degree
        robot->measure_motor_mileage.servo3 = past_total_angle[2] ;//degree       
        
        // std::cerr <<" measure m1 coordinate " << robot->measure_motor_mileage.servo1 <<std::endl;
        // std::cerr <<" measure m2 coordinate " << robot->measure_motor_mileage.servo2<<std::endl;
        
        float  d_len[4];
        d_len[0]= d_past_angle[0] *  degree_to_radian*wheel_radius*0.1;
        d_len[1]= d_past_angle[1] *  degree_to_radian*wheel_radius*0.1;

        // std::cerr <<" measure m1 pos " << d_len[0] <<std::endl;
        // std::cerr <<" measure m2 pos " << d_len[1]<<std::endl;
       
        robot_control.getGlobalCoordinate( d_len , (float* )&robot->measure_global_coordinate);

        // std::cerr <<"X Position  " <<  robot->measure_global_coordinate.x  <<std::endl;
        // std::cerr <<"Y Position  " <<  robot->measure_global_coordinate.y <<std::endl;
        // std::cerr <<"Z Position  " <<  robot->measure_global_coordinate.z  <<std::endl;

}
