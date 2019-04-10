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

//    std::cerr <<"receive doing"  <<std::endl;//setup 
    if( receiveStates_R(rx_byte) )
    {
           
        // std::cerr <<"receive ok"  <<std::endl;//setup 
        //receive a new message
        unsigned char package_update = packageAnalysis();
        if(package_update == 1) {read_analysis_package_count++;
        // std::cerr << "read Out" <<read_analysis_package_count<<std::endl;
        }

        return package_update;
    }
    return 0;
}

unsigned char HFLink_Modbus::byteAnalysisCall_R_FromPAD(const unsigned char rx_byte)
{

//    std::cerr <<"receive doing"  <<std::endl;//setup 
    if( receiveStates_R_from_PAD(rx_byte) )
    {
           
        std::cerr <<"receive from pad ok"  <<std::endl;//setup 
        
        //receive a new message
        unsigned char package_update = packageAnalysis();
        if(package_update == 1) {read_analysis_package_count++;
        std::cerr << "receive from pad count: " <<read_analysis_package_count<<std::endl;
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

    if(robot == NULL){
        printf(" error , the robot is NULL  \n");
        return 0;
    }

    if(rx_message.slave_addr==PAD_INTERFACE_ADDR_CAR1)
    {       
        command_state_=READ_INTERFACE_CAR1_SPEED_CONTROL;

    }
    else if(rx_message.slave_addr==PAD_INTERFACE_ADDR_CAR2)
    {       

         command_state_=READ_INTERFACE_CAR2_SPEED_CONTROL;

    }
    else if(rx_message.slave_addr==PAD_INTERFACE_ADDR_CAR3)
    {       

         command_state_=READ_INTERFACE_CAR3_POSITION_CONTROL;

    }
    else if(rx_message.slave_addr==PAD_INTERFACE_ADDR_CAR4)
    {       
         command_state_=READ_INTERFACE_CAR4_SINGLE_SPEED_CONTROL;
    }
    else if(rx_message.slave_addr==PAD_INTERFACE_ADDR_CAR1_ODOM)
    {       
         command_state_=READ_INTERFACE_CAR1_ODOM_CONTROL;
    }
        else if(rx_message.slave_addr==PAD_INTERFACE_ADDR_ID)
    {       
         command_state_=READ_INTERFACE_ID_CONTROL;
    }
        else if(rx_message.slave_addr==PAD_INTERFACE_ADDR_LASER)
    {       
         command_state_=READ_INTERFACE_LASER_CONTROL;
    }
        else if(rx_message.slave_addr==SENSOR_DATA_TO_PAD_INTERFACE)
    {       
         command_state_=WRITE_SENSOR_DATA_TO_PAD_INTERFACE;
    }
        else if(rx_message.slave_addr==LASER_DATA_TO_PAD_INTERFACE)
    {       
         command_state_=WRITE_LASER_DATA_TO_PAD_INTERFACE;
    }

    unsigned char analysis_state =0;

    switch (command_state_)
    {

    case READ_INTERFACE_CAR1_SPEED_CONTROL:
        analysis_state=readCommandAnalysis(command_state_ ,(short int*)&robot->car1_speed_config , sizeof(robot->car1_speed_config ) );   
        robot->receive_package_flag=1;
        break;
       
    case READ_INTERFACE_CAR2_SPEED_CONTROL:
        analysis_state=readCommandAnalysis(command_state_ ,(short int*)&robot->car1_speed_config , sizeof(robot->car1_speed_config ));   
        robot->receive_package_flag=2;
        break;

    case READ_INTERFACE_CAR3_POSITION_CONTROL:
        analysis_state=readCommandAnalysis(command_state_ ,(short int*)&robot->car3_position_config , sizeof(robot->car3_position_config ) );   
        robot->receive_package_flag=3;
        break;       
    
    case READ_INTERFACE_CAR4_SINGLE_SPEED_CONTROL:
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->car4_single_speed_config , sizeof(robot->car4_single_speed_config ) );   
        robot->receive_package_flag=4;
        break;
    case READ_INTERFACE_CAR1_ODOM_CONTROL:
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->car_global_position_config, sizeof(robot->car_global_position_config ) );   
        robot->receive_package_flag=5;
        break;
    case READ_INTERFACE_ID_CONTROL: 
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->rfid_write_data , sizeof(robot->rfid_write_data ) );   
        robot->receive_package_flag=6;
        break;
    case READ_INTERFACE_LASER_CONTROL:
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->laser_range_config , sizeof(robot->laser_range_config ) );   
        robot->receive_package_flag=7;
        break;               
    
    case WRITE_SENSOR_DATA_TO_PAD_INTERFACE: 
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->ack_to_pad_sensor_data_type , sizeof(robot->ack_to_pad_sensor_data_type ) );   
        robot->receive_package_flag=8;
        break;
    case WRITE_LASER_DATA_TO_PAD_INTERFACE:
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->ack_to_pad_laser_data_type , sizeof(robot->ack_to_pad_laser_data_type ) );   
        robot->receive_package_flag=9;
        break;             
    case WRITE_LASER_DATA_TO_PAD_INTERFACE_1:
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->ack_to_pad_laser_data_type , sizeof(robot->ack_to_pad_laser_data_type ) );   
        robot->receive_package_flag=9;
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
//  SET_CAR4_SPEED_CONTROLed;
//  SET_CAR4_SPEED_CONTROLed ; 
   short int temp_ask_servo1_speed ;
   short int temp_ask_servo2_speed ;
   short int temp_ask_servo3_speed ; 
   short int temp_ask_servo4_speed ; 
   short int temp_ask_servo5_speed ; 
   short int temp_ask_servo6_speed ; 
   short int temp_positionPhaseChange;
   
   
   void *single_command;
   short int v_speed = 1 ;//(m/s)
   float  per_circle_position1 =(360/120)*8*18;

    receive_package_renew[(unsigned char)command_state] = 0 ;

    
    switch (command_state)
    {

    case READ_INTERFACE_CAR1_SPEED_CONTROL:
        sendStructToPAD(PAD_INTERFACE_ADDR_CAR1 , WRITE_MORE_REG,READ_INTERFACE_CAR1_SPEED_CONTROL_ADDR,(unsigned char *)single_command , 0);           
        break;    
    case READ_INTERFACE_CAR2_SPEED_CONTROL:
        sendStructToPAD(PAD_INTERFACE_ADDR_CAR2 , WRITE_MORE_REG,READ_INTERFACE_CAR2_SPEED_CONTROL_ADDR,(unsigned char *)single_command , 0);           
        break;    
    case READ_INTERFACE_CAR3_POSITION_CONTROL:
        sendStructToPAD(PAD_INTERFACE_ADDR_CAR3 , WRITE_MORE_REG,READ_INTERFACE_CAR3_POSITION_CONTROL_ADDR,(unsigned char *)single_command , 0);           
        break;    
    case READ_INTERFACE_CAR4_SINGLE_SPEED_CONTROL:
        sendStructToPAD(PAD_INTERFACE_ADDR_CAR4 , WRITE_MORE_REG,READ_INTERFACE_CAR4_SINGLE_SPEED_CONTROL_ADDR,(unsigned char *)single_command , 0);           
        break;    

    case READ_INTERFACE_CAR1_ODOM_CONTROL:
        sendStructToPAD(PAD_INTERFACE_ADDR_CAR1_ODOM , WRITE_MORE_REG,WRITE_CAR1_ODOM_ACK_TO_PAD_INTERFACE_ADDR,(unsigned char *)single_command , 0);           
        break;    
    case READ_INTERFACE_ID_CONTROL:
        sendStructToPAD(PAD_INTERFACE_ADDR_ID , WRITE_MORE_REG,WRITE_ID_ACK_TO_PAD_INTERFACE_ADDR,(unsigned char *)single_command , 0);           
        break;    
    case READ_INTERFACE_LASER_CONTROL:
        sendStructToPAD(PAD_INTERFACE_ADDR_LASER , WRITE_MORE_REG,WRITE_LASER_ACK_TO_PAD_INTERFACE_ADDR,(unsigned char *)single_command , 0);           
        break;    
   
    case WRITE_SENSOR_DATA_TO_PAD_INTERFACE:        

        tx_to_PAD_buffer[0]=robot->Temperature_Data.read_from_reg_data3;
        tx_to_PAD_buffer[1]=robot->Temperature_Data.read_from_reg_data4; 
        tx_to_PAD_buffer[2]=robot->laxian_length.length_data;
        
        tx_to_PAD_buffer[3]=robot->rfid_read_data.singnal_intensity;
        tx_to_PAD_buffer[4]=robot->rfid_read_data.read_state;
        tx_to_PAD_buffer[5]=robot->rfid_read_data.write_state;

        tx_to_PAD_buffer[6]=robot->rfid_read_data.read_from_reg_data1;
        tx_to_PAD_buffer[7]=robot->rfid_read_data.read_from_reg_data2;
        tx_to_PAD_buffer[8]=robot->rfid_read_data.read_from_reg_data3;
        tx_to_PAD_buffer[9]=robot->rfid_read_data.read_from_reg_data4;
        tx_to_PAD_buffer[10]=robot->rfid_read_data.read_from_reg_data5;
        tx_to_PAD_buffer[11]=robot->rfid_read_data.read_from_reg_data6;
        tx_to_PAD_buffer[12]=robot->rfid_read_data.read_from_reg_data7;
        tx_to_PAD_buffer[13]=robot->rfid_read_data.read_from_reg_data8;


        tx_to_PAD_buffer[14]=robot->euler_angle.pitch;
        tx_to_PAD_buffer[15]=robot->euler_angle.roll; 
        tx_to_PAD_buffer[16]=robot->euler_angle.yaw;
        tx_to_PAD_buffer[17]=robot->bms_battey_.battery_capacity_percentage;  //电池电量百分比
        tx_to_PAD_buffer[18]=robot->bms_battey_.total_voltage;              //电池总电压
        tx_to_PAD_buffer[19]=robot->bms_battey_.current_capacity;          //当前容量
        tx_to_PAD_buffer[20]=robot->bms_battey_.battery_health;          //电池的健康状态
        tx_to_PAD_buffer[21]=robot->moter_error_state_.mot1_error; 
        tx_to_PAD_buffer[22]=robot->moter_error_state_.mot2_error; 
        tx_to_PAD_buffer[23]=robot->moter_error_state_.mot3_error; 
        tx_to_PAD_buffer[24]=robot->moter_error_state_.mot4_error; 
        tx_to_PAD_buffer[25]=robot->moter_error_state_.mot5_error; 
        tx_to_PAD_buffer[26]=robot->moter_error_state_.mot6_error; 
        tx_to_PAD_buffer[27]=robot->moter_speed_state_.mot1_speed; 
        tx_to_PAD_buffer[28]=robot->moter_speed_state_.mot2_speed; 
        tx_to_PAD_buffer[29]=robot->moter_speed_state_.mot3_speed; 
        tx_to_PAD_buffer[30]=robot->moter_speed_state_.mot4_speed; 
        tx_to_PAD_buffer[31]=robot->moter_speed_state_.mot5_speed; 
        tx_to_PAD_buffer[32]=robot->moter_speed_state_.mot6_speed; 
        tx_to_PAD_num=33;
        sendStructToPAD(SENSOR_DATA_TO_PAD_INTERFACE , WRITE_MORE_REG,WRITE_SENSOR_DATA_TO_PAD_INTERFACE_ADDR,(unsigned char *)single_command , 0);      
     
        break;
    case WRITE_LASER_DATA_TO_PAD_INTERFACE:
         
        for(int kk=0;kk<241;kk++) 
        {
            tx_to_PAD_buffer[kk]=robot->laser_scan_data[kk];
        }
        tx_to_PAD_num=241;

        sendStructToPAD(LASER_DATA_TO_PAD_INTERFACE , WRITE_MORE_REG,WRITE_LASER_DATA_TO_PAD_INTERFACE_ADDR,(unsigned char *)single_command , 0);               
        break;
    case WRITE_LASER_DATA_TO_PAD_INTERFACE_1:
        // for(int kk=0;kk<241;kk++) 
        // {
        //     tx_to_PAD_buffer[kk]=robot->laser_scan_data[kk];
        // }
        // tx_to_PAD_num=241;

        sendStructToPAD(LASER_DATA_TO_PAD_INTERFACE , READ_INPUT_REG,WRITE_LASER_DATA_TO_PAD_INTERFACE_ADDR,(unsigned char *)single_command , 0);               
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
*

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

    // }
    return 1;
}

unsigned char HFLink_Modbus::setCommandAnalysis(const MotorModbusCommand command_state , unsigned char* p ,  unsigned short int len)
{ 
    if (hf_link_node_model == 1) 
    { // master  , the slave can set the mastcharer's data ,so this c unsigned short int lenode means received the slave's ack

        printf("I'm master , received a ack ");
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


void HFLink_Modbus::sendStructToPAD(const ModbusSlaveAddr slave_addr,const ModbusCommandCode command_Code, const ModbusCommandRegAddr command_reg_addr , unsigned char* p ,  unsigned short int len)
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
        Request04ToPAD(&tx_message);
        break;

    case WRITE_REG :
        Request06(&tx_message);
  

        break;
        
    case WRITE_MORE_REG :
        Request16ToPAD(&tx_message);
        break;

    default :
        break;
    }
}

