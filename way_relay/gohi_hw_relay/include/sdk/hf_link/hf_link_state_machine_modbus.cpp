/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: hf_link.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* mawenke       2015.10.1   V1.0           creat this file
*
* Description: This file defined hands_free_robot simple communications protocol
*              please read Hands Free Link Manua.doc for detail
***********************************************************************************************************************/

#include "hf_link_state_machine_modbus.h"







unsigned char StateMachineModbus::receiveStates(const unsigned char rx_data)
{
    static unsigned char rev_sum_changed;
    static unsigned char rev_valid_data_num;
    


    if(modbus_receive_state_==0)
    {
        if (rx_data == tx_message.slave_addr)//get slave addr
        {
            modbus_receive_state_ = 1;
            CrcRecValue=0;
            rev_sum_changed=0;
            rx_buffer[0]=rx_data;
            // std::cerr<<"0"<<std::endl;
        }
    }
    else if(modbus_receive_state_==1)
    {
        if (rx_data == tx_message.command_id)
        {
            modbus_receive_state_ = 2;
            rx_buffer[1]=rx_data;
            //  std::cerr<<"1"<<std::endl;
        }
        else if(rx_data == tx_message.command_id+0x80)
        {
            
            std::cerr<<"error status"<<std::endl;
        }
        else 
        {    
            // std::cerr<<"2"<<std::endl;
            // rev_sum_changed=0;
            modbus_receive_state_ = 0;
        }

    } 
    else if(modbus_receive_state_==2)
    {
        if (rx_data == (tx_message.slave_reg_addr)>>8)
        {
            modbus_receive_state_ = 3;
            rx_buffer[2]=rx_data;
            // std::cerr<<"2"<<std::endl;
        }
        else
            modbus_receive_state_ = 0;
    }  

    else if(modbus_receive_state_==3)
    {
        if (rx_data == (tx_message.slave_reg_addr)&0xff)
        {
            modbus_receive_state_ = 4;
            rx_buffer[3]=rx_data;
            // std::cerr<<"3"<<std::endl;
        }
        else
            modbus_receive_state_ = 0;
    }
    else if(modbus_receive_state_==4)
    {
        if (rx_data == tx_message.data[0])
        {
            modbus_receive_state_ = 5;
            rx_buffer[4]=rx_data;
            // std::cerr<<"4"<<std::endl;
        }
        else
            modbus_receive_state_ = 0;
    }
    else if(modbus_receive_state_==5)
    {
        if (rx_data == tx_message.data[1])
        {
            modbus_receive_state_ = 6;
            rx_buffer[5]=rx_data;
            McMBCRC16(rx_buffer,6,&CrcRecValue);
            // std::cerr<<"5"<<std::endl;         
        }
        else
            modbus_receive_state_ = 0;
    }
    else if(modbus_receive_state_==6)
    {          
        if (rx_data == (unsigned char)(CrcRecValue&0xFF))
        {
            modbus_receive_state_ = 7;
            // std::cerr<<"6"<<std::endl;
        }
        else
            modbus_receive_state_ = 0;
    }
    else if(modbus_receive_state_==7)
    {          
        if (rx_data == (unsigned char)(CrcRecValue>>8))
        {
            modbus_receive_state_ = 0;
            receive_message_count ++ ;
            //rx_buffer[0]=0;
            memset(&rx_buffer , 0 , sizeof(rx_buffer));
            memset(&tx_buffer , 0 , sizeof(tx_buffer));  
            // std::cerr<<"7"<<std::endl;
            return 1 ;
        }
        else
            modbus_receive_state_ = 0;
    }        

    return 0;
}

unsigned char StateMachineModbus::receiveStates_R(unsigned char rx_data)
{
   static unsigned char rev_valid_data_num;
   static  unsigned char rev_request_num=0;
        
        if(modbus_receive_state_==0)
        {
            if (rx_data == tx_message.slave_addr)//get slave addr
            {
                modbus_receive_state_ = 1;
                CrcRecValue=0;
                rx_buffer[0]=rx_data;
                rev_valid_data_num=0;
                //   std::cerr<<"00      "<<(int )tx_message.slave_addr<<std::endl;
            }
        }
        else if(modbus_receive_state_==1)
        {
            if (rx_data == tx_message.command_id)
            {
                modbus_receive_state_ = 2;
                rx_buffer[1]=rx_data;
                //  std::cerr<<"11"<<std::endl;
            }
            else if(rx_data == tx_message.command_id+0x80)
            {
                std::cerr<<"error status"<<std::endl;
            }
            else 
            {    
                // rev_sum_changed=0;
                std::cerr<<"other status"<<std::endl;
                modbus_receive_state_ = 0;
            }
        } 
        else if(modbus_receive_state_==2)
        {
            if(tx_message.slave_addr==0x70)     rev_request_num=3;
            else                                rev_request_num=getSendValidData();
             
            if (rx_data == rev_request_num*2)
            {
                modbus_receive_state_ = 3;
                rx_buffer[2]=rx_data;
                //  std::cerr<<"22"<<std::endl;
                rev_valid_data_num=0;
            }
            else
                modbus_receive_state_ = 0;
        }
        else if(modbus_receive_state_==3)
        {                                  
            rx_buffer[3+rev_valid_data_num]=rx_data;
            //  std::cerr<<"33"<<std::endl;   
            if(++rev_valid_data_num>=rx_buffer[2])
            {
                modbus_receive_state_=4;
                //  std::cerr<<"44"<<std::endl;   
                McMBCRC16(rx_buffer,3+rev_valid_data_num,&CrcRecValue);
            }
                   
        }
        else if(modbus_receive_state_==4)
        {          
            if (rx_data == (unsigned char)(CrcRecValue&0xFF))
            {
                modbus_receive_state_ = 5;
                //  std::cerr<<"55"<<std::endl;
            }
            else
            modbus_receive_state_ = 0;
        }
        else if(modbus_receive_state_==5)
        {          
            if (rx_data == (unsigned char)(CrcRecValue>>8))
            {
                modbus_receive_state_ = 0;
                receive_message_count ++ ;

                rx_message.slave_addr=rx_buffer[0];    //clear flag
                rx_message.command_id=rx_buffer[1];
                rx_message.slave_reg_addr=tx_message.slave_reg_addr;

                // rx_message.data[0]=tx_message.slave_reg_addr>>8;
                // rx_message.data[1]=tx_message.slave_reg_addr&0xFF;
                

            //    std::cerr <<"1" <<(int)rx_message.slave_addr<<std::endl;
            //    std::cerr <<"1" <<(int)rx_message.command_id <<std::endl;
            //    std::cerr <<"1" <<(int)rx_message.slave_reg_addr <<std::endl; 
                for (int i=0;i<rx_buffer[2];i++)
                {
                    rx_message.data[i]=rx_buffer[3+rx_buffer[2]-i-1];
                    // std::cerr <<"1" <<(int)rx_message.data[i] <<std::endl;                    
                }
                memset(&rx_buffer , 0 , sizeof(rx_buffer));
                memset(&tx_buffer, 0 , sizeof(tx_buffer));  
                // std::cerr<<"66"<<std::endl;
                return 1 ;
            }
            else
                modbus_receive_state_ = 0;
        }

    return 0;
}


/***********************************************************************************************************************
* Function:    void StateMachine::sendMessage(void)
*
* Scope:
*
* Description:  send a message to hf_link node
*
* Arguments:
*rx_buffer
*rx_buffer
*rx_buffer
* Cpu_Time:    stm32f4+fpu(1 us)
*
* History:
***********************************************************************************************************************/



void StateMachineModbus::Request03(const HFMessageModbus* tx_message_){
    unsigned short int CrcTemp;

    tx_buffer[0]=tx_message_->slave_addr;
    tx_buffer[1]=tx_message_->command_id;
    tx_buffer[2]=tx_message_->slave_reg_addr>>8;
    tx_buffer[3]=tx_message_->slave_reg_addr&0xFF;
    //unsigned short int tx_i  = 0;
    // tx_buffer[4]=tx_message_->data[0]>>8;
    // tx_buffer[5]=tx_message_->data[0]&0xFF;
    //
    tx_buffer[4]=0;
    tx_buffer[5]=1;
    if(tx_message_->slave_addr==0x08)           tx_buffer[5]=2;
    if(tx_message_->slave_addr==0x50)           tx_buffer[5]=3;
    if(tx_message_->slave_reg_addr==0x0024)     tx_buffer[5]=2;
    if(tx_message_->slave_addr==0x09)           tx_buffer[5]=4;


    McMBCRC16(tx_buffer,6,&CrcTemp);
    tx_buffer[6]=CrcTemp&0xFF;
    tx_buffer[7]=CrcTemp>>8;
    send_message_count++;
    tx_buffer_length =8;

}

void StateMachineModbus::Request04(const HFMessageModbus* tx_message_){
    unsigned short int CrcTemp;
    if(tx_message_->slave_addr==0x70)         
    {
        tx_buffer[0]=0x2A;
        tx_buffer[1]=tx_message_->slave_addr;
        tx_buffer[2]=tx_message_->command_id;
        tx_buffer[3]=tx_message_->slave_reg_addr>>8;
        tx_buffer[4]=tx_message_->slave_reg_addr&0xFF;
        //unsigned short int tx_i  = 0;
        // tx_buffer[4]=tx_message_->data[0]>>8;
        // tx_buffer[5]=tx_message_->data[0]&0xFF;
        //
        tx_buffer[5]=0;
        tx_buffer[6]=3;

        
        McMBCRC16(tx_buffer+1,6,&CrcTemp);
        tx_buffer[7]=CrcTemp&0xFF;
        tx_buffer[8]=CrcTemp>>8;
        tx_buffer[9]=0x23;
        
        send_message_count++;
        tx_buffer_length =10;
    }
     //first modify       new 
    else if(tx_message_->slave_addr==0x30)
    {
        tx_buffer[0] =0x3C;
        tx_buffer[1] =0x31;
        tx_buffer[2] =0x35;
        tx_buffer[3] =0x33;
        tx_buffer[4]=tx_message_->slave_addr;
        tx_buffer[5]=tx_message_->slave_addr;
        tx_buffer[6]=0x34;
        tx_buffer[7]=0x32;
        tx_buffer[8]=0x30;
        tx_buffer[9]=0x38;
        tx_buffer[10]=0x35;
        tx_buffer[11]=0x39;
        tx_buffer[12]=0x45;
        tx_buffer[13]=0x37;
        tx_buffer[14]=0x30;
        tx_buffer[15]=0x38;
        tx_buffer[16]=0x46;
        tx_buffer[17]=0x31;
        tx_buffer[18]=0x38;
        tx_buffer[19]=0x30;
        tx_buffer[20]=0x3E;
        send_message_count++;
        tx_buffer_length =21;
    }
    else
    {
        tx_buffer[0]=tx_message_->slave_addr;
        tx_buffer[1]=tx_message_->command_id;
        tx_buffer[2]=tx_message_->slave_reg_addr>>8;
        tx_buffer[3]=tx_message_->slave_reg_addr&0xFF;
        //unsigned short int tx_i  = 0;
        // tx_buffer[4]=tx_message_->data[0]>>8;
        // tx_buffer[5]=tx_message_->data[0]&0xFF;
        //
        tx_buffer[4]=0;
        tx_buffer[5]=1;
        if(tx_message_->slave_addr==0x7F)           tx_buffer[5]=11;
        // if(tx_message_->slave_addr==0x70)           tx_buffer[5]=5; //interface
        
        McMBCRC16(tx_buffer,6,&CrcTemp);
        tx_buffer[6]=CrcTemp&0xFF;
        tx_buffer[7]=CrcTemp>>8;
        send_message_count++;
        tx_buffer_length =8;
    }


}





void StateMachineModbus::Request05(const HFMessageModbus* tx_message_){
    unsigned short int CrcTemp;
    //first modify       new 
    if(tx_message_->slave_addr==0x05)
    {
        tx_buffer[0] =0X01;
        tx_buffer[1] =0X05;
        tx_buffer[2] =0X00;
        tx_buffer[3] =tx_message_->slave_addr;
        tx_buffer[4]=0xff;
        tx_buffer[5]=0x00;

        McMBCRC16(tx_buffer,6,&CrcTemp);
        tx_buffer[6]=CrcTemp&0xFF;
        tx_buffer[7]=CrcTemp>>8;
        
        send_message_count++;
        tx_buffer_length =8;
    }  
    else if(tx_message_->slave_addr==0x06)
    {
        tx_buffer[0] =0X01;
        tx_buffer[1] =0X05;
        tx_buffer[2] =0X00;
        tx_buffer[3] =tx_message_->slave_addr;
        tx_buffer[4]=0xff;
        tx_buffer[5]=0x00;

        McMBCRC16(tx_buffer,6,&CrcTemp);
        tx_buffer[6]=CrcTemp&0xFF;
        tx_buffer[7]=CrcTemp>>8;
        
        send_message_count++;
        tx_buffer_length =8;
    }
    else if(tx_message_->slave_addr==0x07)
    {
        tx_buffer[0] =0X01;
        tx_buffer[1] =0X05;
        tx_buffer[2] =0X00;
        tx_buffer[3] =tx_message_->slave_addr;
        tx_buffer[4]=0xff;
        tx_buffer[5]=0x00;

        McMBCRC16(tx_buffer,6,&CrcTemp);
        tx_buffer[6]=CrcTemp&0xFF;
        tx_buffer[7]=CrcTemp>>8;
        
        send_message_count++;
        tx_buffer_length =8;
    }
    

}



void StateMachineModbus::Request00(const HFMessageModbus* tx_message_){
    unsigned short int CrcTemp;
    //first modify       new 
    if(tx_message_->slave_addr==0x05)
    {
        tx_buffer[0] =0X01;
        tx_buffer[1] =0X05;
        tx_buffer[2] =0X00;
        tx_buffer[3] =tx_message_->slave_addr;
        tx_buffer[4]=0x00;
        tx_buffer[5]=0x00;

        McMBCRC16(tx_buffer,6,&CrcTemp);
        tx_buffer[6]=CrcTemp&0xFF;
        tx_buffer[7]=CrcTemp>>8;
        
        send_message_count++;
        tx_buffer_length =8;
    }  
    else if(tx_message_->slave_addr==0x06)
    {
        tx_buffer[0] =0X01;
        tx_buffer[1] =0X05;
        tx_buffer[2] =0X00;
        tx_buffer[3] =tx_message_->slave_addr;
        tx_buffer[4]=0x00;
        tx_buffer[5]=0x00;

        McMBCRC16(tx_buffer,6,&CrcTemp);
        tx_buffer[6]=CrcTemp&0xFF;
        tx_buffer[7]=CrcTemp>>8;
        
        send_message_count++;
        tx_buffer_length =8;
    }
    else if(tx_message_->slave_addr==0x07)
    {
        tx_buffer[0] =0X01;
        tx_buffer[1] =0X05;
        tx_buffer[2] =0X00;
        tx_buffer[3] =tx_message_->slave_addr;
        tx_buffer[4]=0x00;
        tx_buffer[5]=0x00;

        McMBCRC16(tx_buffer,6,&CrcTemp);
        tx_buffer[6]=CrcTemp&0xFF;
        tx_buffer[7]=CrcTemp>>8;
        
        send_message_count++;
        tx_buffer_length =8;
    }
    

}
void StateMachineModbus::Request06(const HFMessageModbus* tx_message_){

    unsigned short int  CrcTemp;

    tx_buffer[0]=tx_message_->slave_addr;
    tx_buffer[1]=tx_message_->command_id;
    tx_buffer[2]=tx_message_->slave_reg_addr>>8;
    tx_buffer[3]=tx_message_->slave_reg_addr&0xFF;
    unsigned short int tx_i  = 0;
    tx_buffer[5]=tx_message_->data[tx_i];
    tx_buffer[4]=tx_message_->data[tx_i+1];
    // std::cerr <<"bb" <<(int)tx_buffer[4] <<std::endl;
    // std::cerr <<"cc" <<(int)tx_buffer[5] <<std::endl;
    
    McMBCRC16(tx_buffer,6,&CrcTemp);
    tx_buffer[6]=CrcTemp&0xFF;
    tx_buffer[7]=CrcTemp>>8;
    tx_buffer_length =8;
    send_message_count++;
}
void StateMachineModbus::Request16(const HFMessageModbus* tx_message_, unsigned short int len){

    unsigned short int  i;
    unsigned short int  LongSen;

    unsigned short int  CrcTemp;

    tx_buffer[0]=tx_message_->slave_addr;
    tx_buffer[1]=tx_message_->command_id;
    tx_buffer[2]=tx_message_->slave_reg_addr>>8;
    tx_buffer[3]=tx_message_->slave_reg_addr&0xFF;



    unsigned short int tx_i  = 0;

    if(tx_message_->slave_addr==0x7F)      
    {
        tx_buffer[4]=0x00; //regNum=4; //2 bytes
        tx_buffer[5]=len/2;
        tx_buffer[6]=len;


        tx_buffer[7]=tx_message_->data[1]; //write reg data1
        tx_buffer[8]=tx_message_->data[0]; 
        
        tx_buffer[9]=tx_message_->data[3]; //write reg data2
        tx_buffer[10]=tx_message_->data[2];
        
        tx_buffer[11]=tx_message_->data[5];//write reg data3
        tx_buffer[12]=tx_message_->data[4];

        tx_buffer[13]=tx_message_->data[7];//write reg data4
        tx_buffer[14]=tx_message_->data[6];

        tx_buffer[15]=tx_message_->data[9];//write reg data5
        tx_buffer[16]=tx_message_->data[8];

        tx_buffer[17]=tx_message_->data[11];//write reg data6
        tx_buffer[18]=tx_message_->data[10];

        tx_buffer[19]=tx_message_->data[13];//write reg data7
        tx_buffer[20]=tx_message_->data[12];

        tx_buffer[21]=tx_message_->data[15];//write reg data8
        tx_buffer[22]=tx_message_->data[14];
    
        

        McMBCRC16(tx_buffer,23,&CrcTemp);
        tx_buffer[23]=CrcTemp&0xFF;
        tx_buffer[24]=CrcTemp>>8;
        tx_buffer_length =25;
        send_message_count++;
    }
    else   if(tx_message_->slave_addr==0x03) 
    {
        tx_buffer[4]=0x00; //regNum=4; //2 bytes
        tx_buffer[5]=len/2;
        tx_buffer[6]=len;
        tx_buffer[7]=tx_message_->data[1]; //positionPhaseChange=500*10;// 2 bytes
        tx_buffer[8]=tx_message_->data[0]; 
        
        tx_buffer[9]=tx_message_->data[3];  //positiontype=0;// 2 bytes
        tx_buffer[10]=tx_message_->data[2];
        
        tx_buffer[11]=tx_message_->data[5];//position=0xfff;// 4 bytes
        tx_buffer[12]=tx_message_->data[4];

        tx_buffer[13]=tx_message_->data[7];
        tx_buffer[14]=tx_message_->data[6];      

        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[0]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[1]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[2]<<std::endl;
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[3]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[4]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[5]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[6]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[7]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[8]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[9]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[10]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[11]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[12]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[13]<<std::endl; 
        // std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[14]<<std::endl; 
        
        

        McMBCRC16(tx_buffer,15,&CrcTemp);
        tx_buffer[15]=CrcTemp&0xFF;
        tx_buffer[16]=CrcTemp>>8;
        
        std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[15]<<std::endl; 
        std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[16]<<std::endl; 

        tx_buffer_length =9+8;
        send_message_count++;
    }
    else   if(tx_message_->slave_addr==0x05) 
    {
        tx_buffer[4]=0x00; //regNum=4; //2 bytes
        tx_buffer[5]=len/2;
        tx_buffer[6]=len;
        tx_buffer[7]=tx_message_->data[1]; //positionPhaseChange=500*10;// 2 bytes
        tx_buffer[8]=tx_message_->data[0]; 
        
        tx_buffer[9]=tx_message_->data[3];  //positiontype=0;// 2 bytes
        tx_buffer[10]=tx_message_->data[2];
        
        tx_buffer[11]=tx_message_->data[5];//position=0xfff;// 4 bytes
        tx_buffer[12]=tx_message_->data[4];

        tx_buffer[13]=tx_message_->data[7];
        tx_buffer[14]=tx_message_->data[6];      

      
        

        McMBCRC16(tx_buffer,15,&CrcTemp);
        tx_buffer[15]=CrcTemp&0xFF;
        tx_buffer[16]=CrcTemp>>8;
        
        std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[15]<<std::endl; 
        std::cerr <<"read_from_reg_data1  " <<(short int)tx_buffer[16]<<std::endl; 

        tx_buffer_length =9+8;
        send_message_count++;
    }

    
    

}






// CRC MODBUS 效验    // tx_buffer_length =8;
// 输入参数: pDataIn: 数据地址
//           iLenIn: 数据长度
// 输出参数: pCRCOut: 2字节校验值
void StateMachineModbus::McMBCRC16(unsigned char *pDataIn, int iLenIn, unsigned short int  *pCRCOut){
    int ucCRCHi = 0xFF;
    int ucCRCLo = 0xFF;
    int iIndex;
    while(iLenIn-- )
    {
        iIndex = ucCRCLo ^ *( pDataIn++ );
        ucCRCLo = ( int )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    *pCRCOut = ( ucCRCHi << 8 | ucCRCLo );
}



void StateMachineModbus::HexToAscii(unsigned char * pHex, unsigned char * pAscii, int nLen)
 {
    unsigned char Nibble[2];  
    for (int i = 0; i < nLen; i++)  
    { 
        Nibble[0] = (pHex[i] & 0xF0) >> 4;  
        Nibble[1] = pHex[i] & 0x0F;   
        for (int j = 0; j < 2; j++) 
        { 
            if (Nibble[j] < 10)                Nibble[j] += 0x30;   
            else  
            {          
                if (Nibble[j] < 16)            Nibble[j] = Nibble[j] - 10 + 'A';
            } 
            *pAscii++ = Nibble[j]; 
        }   // for (int j = ...)    
    }   // for (int i = ...)
}
       

