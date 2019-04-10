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
        if(package_update == 1) {write_analysis_package_count++;        
        //std::cerr << "write Out" <<write_analysis_package_count<<std::endl;
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
        if(package_update == 1) {read_analysis_package_count++;
        //std::cerr << "read Out" <<read_analysis_package_count<<std::endl;
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


   short int temp_mea_servo3_speed ;
   short int temp_mea_servo4_speed ; 

   int temp_mea_motor3_position;
   int temp_mea_motor4_position;



  
    float  per_circle_position =(360/120)*8*60; //将30改为60
    float  per_circle_position1 =(360/120)*8*15;

    float  pid_t  =0.1;

    // float per_meter_odomey = 0;
    float meter_flag = 0;

    if(robot == NULL){
        printf(" error , the robot is NULL  \n");
        return 0;
    }
   
    if(rx_message.slave_addr==MOTOR3_ADDR)
    {
        if(rx_message.slave_reg_addr==READ_ERROR_STATE_ADDR)                     command_state_=READ_MOT3_ERROR_STATE;        
        else if(rx_message.slave_reg_addr==READ_MOT_SPEED_ADDR)                  command_state_=READ_MOT3_SPEED;
        else if(rx_message.slave_reg_addr==SET_MOT_SPEED_ADDR)                   command_state_=SET_STAIR_CAR2_SPEED_CONTROL;
        else if(rx_message.slave_reg_addr==READ_POSITION_COMPLETE_STATE_ADDR)    command_state_=READ_CAR2_MOTOR3_COMPLETE_STATE;
        else if(rx_message.slave_reg_addr==READ_MOT_POSITION_ADDR)               command_state_=READ_MOT3_REAL_POSITION;         
        else if(rx_message.slave_reg_addr==SET_BRAKE_STATE_ADDR)                 command_state_=SET_MOT3_BRAKE_STATE;
        else if(rx_message.slave_reg_addr==SET_MOT_POSITION_ADDR)                command_state_=SET_CAR2_POSITION_CONTROL;
        else if(rx_message.slave_reg_addr==READ_POSITION_RESET_STATE_ADDR)       command_state_=READ_CAR2_MOTER3_RESET_STATE;
        else if(rx_message.slave_reg_addr==SET_MOT3_RESET_TEST_ADDR)             command_state_=SET_MOT3_RESET_TEST;
        else                                                                     command_state_=LAST_COMMAND_FLAG_;
    }
    else if(rx_message.slave_addr==MOTOR4_ADDR)
    {
        if(rx_message.slave_reg_addr==READ_ERROR_STATE_ADDR)                     command_state_=READ_MOT4_ERROR_STATE;        
        else if(rx_message.slave_reg_addr==READ_MOT_SPEED_ADDR)                  command_state_=READ_MOT4_SPEED;
        else if(rx_message.slave_reg_addr==READ_POSITION_COMPLETE_STATE_ADDR)    command_state_=READ_CAR2_MOTOR4_COMPLETE_STATE;
        else if(rx_message.slave_reg_addr==READ_MOT_POSITION_ADDR)               command_state_=READ_MOT4_REAL_POSITION;         
        else if(rx_message.slave_reg_addr==SET_BRAKE_STATE_ADDR)                 command_state_=SET_MOT4_BRAKE_STATE;
        else if(rx_message.slave_reg_addr==SET_MOT_SPEED_ADDR)                   command_state_=SET_CAR2_SPEED_CONTROL;
        else                                                                     command_state_=LAST_COMMAND_FLAG_;
    }



    unsigned char analysis_state =0;

    switch (command_state_)
    {

    case READ_MOT3_ERROR_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->motor_error_state.error3 , sizeof(robot->motor_error_state.error3));
        break;
    case READ_MOT4_ERROR_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->motor_error_state.error4 , sizeof(robot->motor_error_state.error4));
        break;

    case READ_CAR2_MOTOR3_COMPLETE_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (short int* )&robot->motor_pos_comp_state.posComp3 ,     sizeof(robot->motor_pos_comp_state.posComp3));
        stair_position_complete_state_temp =(unsigned short)robot->motor_pos_comp_state.posComp3;  //first modify 
        
        if(robot->motor_pos_comp_state.posComp3)
        {
            robot->motor_pos_comp_state.posComp3=0;
            robot->stair_positionPhaseChange=0;                
        }


        break;
    case READ_CAR2_MOTER3_RESET_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (short int* )&robot->stair_SQ_reset_state , sizeof(robot->stair_SQ_reset_state ));
        stair_reset_SQ_state_temp  =robot->stair_SQ_reset_state;
        // std::cerr <<"READ_CAR2_MOTOR3_RESET_STATE " <<(int)robot->stair_SQ_reset_state<<std::endl;
        if(robot->stair_SQ_reset_state)
        {
            robot->stair_SQ_reset_state=0;             
        }


        break;
    case SET_MOT3_RESET_TEST :
        analysis_state=readCommandAnalysis(command_state_ , (short int* )&robot->stair_SQ_reset_test, sizeof(robot->stair_SQ_reset_test ));
        stair_reset_SQ_state_temp  =robot->stair_SQ_reset_state;
        break;
 
    case READ_CAR2_MOTOR4_COMPLETE_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->motor_pos_comp_state.posComp4 , sizeof(robot->motor_pos_comp_state.posComp4));
        break;



    case READ_MOT3_REAL_POSITION :      
        // d_past_angle[1] =0;            
        analysis_state=readCommandAnalysis(command_state_ , (int* )&temp_mea_motor3_position , sizeof(robot->ask_measure_motor_position.position3));
        robot->ask_measure_motor_position.position3=(float)temp_mea_motor3_position;

        robot->ask_measure_motor_position_dif.position3=robot->ask_measure_motor_position.position3-robot->ask_measure_motor_position_last.position3;
        robot->ask_measure_motor_position_last.position3=robot->ask_measure_motor_position.position3;
        //recording total angle for robot coordinate calculation
        d_past_angle[0] = (robot->ask_measure_motor_position_dif.position3/per_circle_position)*360;
        past_total_angle[0]+= ( robot->ask_measure_motor_position_dif.position3/per_circle_position)*360;

        //calc motor speed  degree/s
        robot->ask_measure_motor_speed.servo3=   robot->ask_measure_motor_position_dif.position3 * 360 / ( per_circle_position*pid_t )*degree_to_radian;
        // std::cerr <<"get servo3 speed  " <<robot->ask_measure_motor_speed.servo3<<std::endl;
        break;
    case READ_MOT4_REAL_POSITION :      
        // d_past_angle[1] =0;            
        analysis_state=readCommandAnalysis(command_state_ , (int* )&temp_mea_motor4_position , sizeof(robot->ask_measure_motor_position.position4));
        robot->ask_measure_motor_position.position4=(float)temp_mea_motor4_position;

        robot->ask_measure_motor_position_dif.position4=robot->ask_measure_motor_position.position4-robot->ask_measure_motor_position_last.position4;
        robot->ask_measure_motor_position_last.position4=robot->ask_measure_motor_position.position4;
        //recording total angle for robot coordinate calculation
        d_past_angle[1] = (robot->ask_measure_motor_position_dif.position4/per_circle_position1)*360;
        past_total_angle[1]+= ( robot->ask_measure_motor_position_dif.position4/per_circle_position1)*360;

        //calc motor speed  degree/s
        robot->ask_measure_motor_speed.servo4=   robot->ask_measure_motor_position_dif.position4 * 360 / ( per_circle_position1*pid_t )*degree_to_radian;
        // std::cerr <<"get servo4 speed  " <<robot->ask_measure_motor_speed.servo4<<std::endl;
        break;

    case SET_STAIR_CAR2_SPEED_CONTROL :
        analysis_state=setCommandAnalysis(command_state_ , (float* )&robot->ask_expect_motor_speed.servo3 , sizeof(robot->ask_expect_motor_speed.servo3) );
        break;
    case SET_MOT3_BRAKE_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->ask_brake_config.brake3 , sizeof(robot->ask_brake_config.brake3));
        break;  
    case SET_MOT4_BRAKE_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->ask_brake_config.brake4 , sizeof(robot->ask_brake_config.brake4));
        break;  
   
    case SET_CAR2_POSITION_CONTROL :
        analysis_state=setCommandAnalysis(command_state_ , (float* )&robot->ask_expect_motor_speed.servo3 , sizeof(robot->ask_expect_motor_speed.servo3) );
        break;   
    case SET_CAR2_SPEED_CONTROL :
        analysis_state=setCommandAnalysis(command_state_ , (float* )&robot->ask_expect_motor_speed.servo4 , sizeof(robot->ask_expect_motor_speed.servo4) );
        break;


    default :
        analysis_state = 0;
        break;

    }

    // rx_message.slave_addr=0;    //clear flag
    // rx_message.command_id=0;
    // rx_message.slave_reg_addr=0;
    // memset(&rx_message.data , 0 , sizeof(rx_message.data));


    // tx_message.slave_addr=0;    //clear flag
    // tx_message.command_id=0;
    // tx_message.slave_reg_addr=0;
    // memset(&tx_message.data , 0 , sizeof(tx_message.data));


    return analysis_state;
}



unsigned char HFLink_Modbus::masterSendCommand(const MotorModbusCommand command_state,int relay_on_or_off)
{
   void *single_command;
   short int v_speed = 1 ;//(m/s

   receive_package_renew[(unsigned char)command_state] = 0 ;


    switch (command_state)
    {
        case SET_RELAY5_STATE:  //设置继电器状态
            // std::cerr<<"into relay5"<<std::endl;
            if(relay_on_or_off & 0x20){
                sendStruct(SET_RELAY5_ADDR , WRITE_RELAY_REG_ON,SET_RELAY5_REG_ADDR,(unsigned char *)single_command , 0);           
            }else
                sendStruct(SET_RELAY5_ADDR , WRITE_RELAY_REG_OFF,SET_RELAY5_REG_ADDR,(unsigned char *)single_command , 0);     
            break; 
        case SET_RELAY6_STATE:  //设置继电器状态
            // std::cerr<<"into relay6"<<std::endl;
            if(relay_on_or_off & 0x40)
                sendStruct(SET_RELAY6_ADDR , WRITE_RELAY_REG_ON,SET_RELAY6_REG_ADDR,(unsigned char *)single_command , 0);           
            else 
                sendStruct(SET_RELAY6_ADDR , WRITE_RELAY_REG_OFF,SET_RELAY6_REG_ADDR,(unsigned char *)single_command , 0);           
            break; 
        case SET_RELAY7_STATE:  //设置继电器状态
            // std::cerr<<"into relay7"<<std::endl;
            if(relay_on_or_off & 0x80)
                sendStruct(SET_RELAY7_ADDR , WRITE_RELAY_REG_ON,SET_RELAY7_REG_ADDR,(unsigned char *)single_command , 0);           
            else 
                sendStruct(SET_RELAY7_ADDR , WRITE_RELAY_REG_OFF,SET_RELAY7_REG_ADDR,(unsigned char *)single_command , 0);  
            break; 

   default : // case SET_CAR5_SPEED_CONTROL :  

        break;
    } 
    return 1;
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

   short int temp_ask_servo3_speed ; 
   short int temp_ask_servo4_speed ; 

   short int temp_positionPhaseChange;
   
   
   void *single_command;

   float  per_circle_position =(360/120)*8*60;   //将30改为60
   float  per_circle_position1 =(360/120)*8*15;


    receive_package_renew[(unsigned char)command_state] = 0 ;

    
    switch (command_state)
    {
  
    case READ_MOT3_ERROR_STATE :
        sendStruct(MOTOR3_ADDR , READ_REG,READ_ERROR_STATE_ADDR,(unsigned char *)single_command , 0 );
        break;                         
    case READ_MOT4_ERROR_STATE :
        sendStruct(MOTOR4_ADDR , READ_REG,READ_ERROR_STATE_ADDR,(unsigned char *)single_command , 0 );
        break;
    


    case READ_MOT3_SPEED :
        sendStruct(MOTOR3_ADDR , READ_REG,READ_MOT_SPEED_ADDR,(unsigned char *)single_command , 0);
        break;
    case READ_MOT4_SPEED :
        sendStruct(MOTOR4_ADDR , READ_REG,READ_MOT_SPEED_ADDR,(unsigned char *)single_command , 0);
        break;



    case READ_MOT3_REAL_POSITION :
        sendStruct(MOTOR3_ADDR , READ_REG,READ_MOT_POSITION_ADDR,(unsigned char *)single_command , 0);
        break;
    case READ_MOT4_REAL_POSITION :
        sendStruct(MOTOR4_ADDR , READ_REG,READ_MOT_POSITION_ADDR,(unsigned char *)single_command , 0);
        break;


    case READ_CAR2_MOTOR3_COMPLETE_STATE :
        // std::cerr <<"ask servo3 state " <<(int)robot->motor_pos_comp_state.posComp3<<std::endl;
        sendStruct(MOTOR3_ADDR , READ_REG,READ_POSITION_COMPLETE_STATE_ADDR,(unsigned char *)single_command , 0);              
        break;  
    case READ_CAR2_MOTER3_RESET_STATE :
        // std::cerr <<"ask servo3 state " <<(int)robot->motor_pos_comp_state.posComp3<<std::endl;
         sendStruct(MOTOR3_ADDR , READ_REG,READ_POSITION_RESET_STATE_ADDR,(unsigned char *)single_command , 0);              
        break;      
    case READ_CAR2_MOTOR4_COMPLETE_STATE :
        sendStruct(MOTOR4_ADDR , READ_REG,READ_POSITION_COMPLETE_STATE_ADDR,(unsigned char *)single_command , 0);              
        break;


    case SET_MOT3_BRAKE_STATE :
        robot->ask_brake_config.brake3 =1;   //紧急停止刹车
        sendStruct(MOTOR3_ADDR , WRITE_REG,SET_BRAKE_STATE_ADDR, (unsigned char *)&robot->ask_brake_config.brake3 , sizeof(robot->ask_brake_config.brake3) );  
        break;
    case SET_MOT3_FREE_BRAKE_STATE :
        robot->ask_brake_config.brake3 =2;   //自由停止刹车
        sendStruct(MOTOR3_ADDR , WRITE_REG,SET_BRAKE_STATE_ADDR, (unsigned char *)&robot->ask_brake_config.brake3 , sizeof(robot->ask_brake_config.brake3) );  
         break;
    case SET_MOT3_NORMAL_BRAKE_STATE :
        robot->ask_brake_config.brake3 =0;   //正常停止刹车
        sendStruct(MOTOR3_ADDR , WRITE_REG,SET_BRAKE_STATE_ADDR, (unsigned char *)&robot->ask_brake_config.brake3 , sizeof(robot->ask_brake_config.brake3) );  
         break;
    case SET_MOT4_BRAKE_STATE :   //正常停止刹车
         robot->ask_brake_config.brake4 =1;   //紧急停止刹车
        sendStruct(MOTOR4_ADDR , WRITE_REG,SET_BRAKE_STATE_ADDR, (unsigned char *)&robot->ask_brake_config.brake4 , sizeof(robot->ask_brake_config.brake4) );     
        break;        
    case SET_MOT3_RESET_TEST :
        robot->stair_SQ_reset_test =3;
        sendStruct(MOTOR3_ADDR , WRITE_REG,SET_MOT3_RESET_TEST_ADDR, (unsigned char *)&robot->stair_SQ_reset_test , sizeof(robot->stair_SQ_reset_test) );     
        break;          
    case SET_CAR2_POSITION_CONTROL :
        robot->ask_position_set.positionPhaseChange=robot->stair_positionPhaseChange;  //rad/s
        robot->ask_position_set.positiontype= robot->stair_type ;//rad
        robot->ask_position_set.position=robot->stair_position;

        // stair_position_temp_temp =robot->stair_position;  //first modify 
        // std::cerr <<"stair ask  positionPhaseChange " <<robot->ask_position_set.positionPhaseChange <<std::endl;
        // std::cerr <<"stair ask  type " <<robot->ask_position_set.positiontype <<std::endl;
        // std::cerr <<"stair ask  position " <<robot->ask_position_set.position <<std::endl;
        
        robot->ask_position_set.positionPhaseChange=(robot->ask_position_set.positionPhaseChange)*60/2/3.14;
        robot->ask_position_set.positionPhaseChange=robot->ask_position_set.positionPhaseChange*8*60/0.1/20; //将30改为60
        robot->ask_position_config.positionPhaseChange      =(short int)robot->ask_position_set.positionPhaseChange;  
        // robot->ask_position_config.positionPhaseChange      =750;
        // std::cerr<<"robot->ask_position_config.positionPhaseChange is "<<robot->ask_position_config.positionPhaseChange<<std::endl;
        robot->ask_position_config.positiontype             =robot->ask_position_set.positiontype;  
        robot->ask_position_set.position=(robot->ask_position_set.position*radian_to_degree*per_circle_position)/360;
        robot->ask_position_config.positionH=robot->ask_position_set.position>>16;   
        robot->ask_position_config.positionL=robot->ask_position_set.position&0xFFFF;   


        // std::cerr <<"ask  position PhaseChange" <<robot->ask_position_config.positionPhaseChange <<std::endl;
        // std::cerr <<"ask  position positiontype" <<robot->ask_position_config.positiontype <<std::endl;
        // std::cerr <<"ask  position H" <<robot->ask_position_config.positionH <<std::endl;
        // std::cerr <<"ask  position L" <<robot->ask_position_config.positionL <<std::endl;
        

        sendStruct(MOTOR3_ADDR , WRITE_MORE_REG,SET_MOT_POSITION_ADDR,(unsigned char* )&robot->ask_position_config , sizeof(robot->ask_position_config) );
        break;

    case SET_STAIR_CAR2_SPEED_CONTROL :
        
        robot->ask_expect_motor_speed.servo3=robot->stair_speed;
        robot->ask_expect_motor_speed.servo3=(robot->ask_expect_motor_speed.servo3)*60/2/3.14;
        robot->ask_expect_motor_speed.servo3=robot->ask_expect_motor_speed.servo3*8*60/0.1/20;
        temp_ask_servo3_speed     =(short int )robot->ask_expect_motor_speed.servo3;      
        sendStruct(MOTOR3_ADDR , WRITE_REG,SET_MOT_SPEED_ADDR,(unsigned char *)&temp_ask_servo3_speed, sizeof(temp_ask_servo3_speed) );              
        break;


    case SET_CAR2_SPEED_CONTROL :
        
        robot->ask_expect_motor_speed.servo4=robot->dilong_speed;
        robot->ask_expect_motor_speed.servo4=(robot->ask_expect_motor_speed.servo4)*60/2/3.14;
        robot->ask_expect_motor_speed.servo4=robot->ask_expect_motor_speed.servo4*8*15/0.1/20;
        temp_ask_servo4_speed     =(short int )robot->ask_expect_motor_speed.servo4;      
        sendStruct(MOTOR4_ADDR , WRITE_REG,SET_MOT_SPEED_ADDR,(unsigned char *)&temp_ask_servo4_speed, sizeof(temp_ask_servo4_speed) );              
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
    { // master   , means the slave feedback a package to master , and the master save this package

        memcpy(p , &rx_message.data[0] , len);     

        receive_package_renew[(unsigned char)command_state] = 1 ;
    }

    return 1;
}


unsigned char HFLink_Modbus::setCommandAnalysis(const MotorModbusCommand command_state , float* p ,  unsigned short int len)
{

    if (hf_link_node_model == 1)
    { // master  , the slave can set the master's data ,so this code means received the slave's ack

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
* Description:
* len =0       send a Single command to the friend
*              if i am slave , it can be  feed back a ack to master or request instructions  like SHAKING_HANDS
*              if i am master , it can be s             ome request instructions like READ_ROBOT_SYSTEM_INFO READ_xxx
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
    case WRITE_RELAY_REG_ON:
        Request05(&tx_message);

        break;
    case WRITE_RELAY_REG_OFF:
       Request00(&tx_message);
        break;
    case WRITE_MORE_REG :
        Request16(&tx_message,len);
        break;

    default :
        break;
    }
}


