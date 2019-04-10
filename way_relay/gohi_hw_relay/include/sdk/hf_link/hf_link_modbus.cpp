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
    unsigned char analysis_state;

    short int temp_laxian_length ; 
    short int temp_euler_angle[3] ; 

    short int temp_RFID_X_Y_SPEED;  //RFID数据//
    



  
    float  per_circle_position =(360/120)*8*30;
    float  pid_t  =0.1;



    if(robot == NULL){
        printf(" error , the robot is NULL  \n");
        return 0;
    }



    if(rx_message.slave_addr==LAXIAN_ADDR)
    {
        if(rx_message.slave_reg_addr==READ_LAXIAN_POSITION_ADDR)                 command_state_=READ_LAXIAN_POSITION;
    }
    else if(rx_message.slave_addr==IMU_ADDR)
    {          
        if(rx_message.slave_reg_addr==READ_EULER_ANGLE_ADDR)                     command_state_=READ_EULER_ANGLE;
    }
    else if(rx_message.slave_addr==RFID_ADDR)
    {       
        if(rx_message.slave_reg_addr==READ_RFID_REG_DATA_ADDR)                   command_state_=READ_RFID_REG_DATA;
    }

    else if(rx_message.slave_addr==THERMOMETER_ADDR)   //--------------------温度传感器
    {       
        if(rx_message.slave_reg_addr==READ_THERMOMETER_REG_DATA_ADDR)                   command_state_=READ_THERMOMETER_REG_DATA;
    }
   


    switch (command_state_)
    {


    case READ_LAXIAN_POSITION:
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&temp_laxian_length , sizeof(robot->laxian_length.length_data) );   
        robot->laxian_length.length_data=temp_laxian_length;
        // std::cerr <<"measure Length: " <<temp_laxian_length<<"  mm" <<std::endl; 
        break;
    case READ_EULER_ANGLE:
        // std::cerr <<"yyy3"  <<std::endl;//setup 
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->euler_angle , sizeof(robot->euler_angle) );   
        // std::cerr <<"measure pitch  " <<robot->euler_angle.pitch*180.0/32768  <<std::endl;  
        // std::cerr <<"measure roll  " <<robot->euler_angle.roll *180.0/32768<<std::endl;   
        // std::cerr <<"measure yaw  " <<robot->euler_angle.yaw*180.0/32768<<std::endl;   
        
        break;

    case READ_RFID_REG_DATA:
  
       // analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&temp_RFID_X_Y_SPEED , sizeof(robot->RFIDData. x_y_speed) ); 
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->rfid_read_data , sizeof(robot->rfid_read_data ));   
        // std::cerr <<"write_state  " <<(short int)robot->rfid_read_data.write_state<<std::endl; 
        // std::cerr <<"read_state  " <<(short int)robot->rfid_read_data.read_state<<std::endl; 
        // std::cerr <<"singnal_intensity  " <<(short int)robot->rfid_read_data.singnal_intensity<<std::endl; 

        // std::cerr <<"read_from_reg_data1  " <<(short int)robot->rfid_read_data.read_from_reg_data1<<std::endl; 
        // std::cerr <<"read_from_reg_data2  " <<(short int)robot->rfid_read_data.read_from_reg_data2<<std::endl; 
        // std::cerr <<"read_from_reg_data3  " <<(short int)robot->rfid_read_data.read_from_reg_data3<<std::endl; 
        // std::cerr <<"read_from_reg_data4  " <<(short int)robot->rfid_read_data.read_from_reg_data4<<std::endl; 
        // std::cerr <<"read_from_reg_data5  " <<(short int)robot->rfid_read_data.read_from_reg_data5<<std::endl; 
        // std::cerr <<"read_from_reg_data6  " <<(short int)robot->rfid_read_data.read_from_reg_data6<<std::endl; 
        // std::cerr <<"read_from_reg_data7  " <<(short int)robot->rfid_read_data.read_from_reg_data7<<std::endl; 
        // std::cerr <<"read_from_reg_data8  " <<(short int)robot->rfid_read_data.read_from_reg_data8<<std::endl; 

        break;

     case READ_THERMOMETER_REG_DATA://读温度传感器
 
        analysis_state=readCommandAnalysis(command_state_ , (short int*)&robot->Temperature_Data , sizeof(robot->Temperature_Data ));   
        // std::cerr <<"温度传感器1路  " <<(short int)robot->Temperature_Data.read_from_reg_data1/10<<" 摄氏度 "std::endl; 
        // std::cerr <<"温度传感器2路  " <<(short int)robot->Temperature_Data.read_from_reg_data2/10<<"摄氏度  "std::endl; 
        // std::cerr <<"温度传感器3路: " <<(float)robot->Temperature_Data.read_from_reg_data3/10<<" 摄氏度  "<<std::endl; 
        // std::cerr <<"温度传感器4路: " <<(float)robot->Temperature_Data.read_from_reg_data4/10<<" 摄氏度  "<<std::endl; 

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
   void *single_command;
   short int v_speed = 1 ;//(m/s)
   float  per_circle_position1 =(360/120)*8*18;

    receive_package_renew[(unsigned char)command_state] = 0 ;

    
    switch (command_state)
    {
        case READ_BMS_REG_DATA:  //读BMS

            sendStruct(BMS_ADDR , READ_INPUT_REG ,READ_BMS_REG_DATA_ADDR,(unsigned char *)single_command , 0);           
            break; 

   default : // case SET_CAR5_SPEED_CONTROL :  

        break;
    } 
    return 1;
}


unsigned char HFLink_Modbus::masterSendCommand(const MotorModbusCommand command_state,int relay_on_or_off)
{
   void *single_command;
   short int v_speed = 1 ;//(m/s)
   float  per_circle_position1 =(360/120)*8*18;

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
* len>0 :      send a Struct command to      // robot->expect_robot_speed.x=0.1;READ_RFID_REG_DATA

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




