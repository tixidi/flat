#ifndef HF_LINK_STATE_MACHINE_MODBUS_H
#define HF_LINK_STATE_MACHINE_MODBUS_H

#include "stdio.h"
#include <tabcrc.h>
#include <iostream>
#include <string.h> 

//limite one message's size
static const unsigned short int MODBUS_MESSAGE_BUFER_SIZE = 1024;

typedef struct HFMessageModbus{
    unsigned char slave_addr;
    unsigned char command_id;
    unsigned short int slave_reg_addr;
    unsigned char data[MODBUS_MESSAGE_BUFER_SIZE];
}HFMessageModbus;

//communication status
enum RecstateModbus{
    WAITING_FF1_,
    WAITING_FF2_,
    SENDER_ID_,
    RECEIVER_ID_,
    RECEIVE_LEN_H_,
    RECEIVE_LEN_L_,
    RECEIVE_PACKAGE_,
    RECEIVE_CHECK_
};

class StateMachineModbus
{
public:
    StateMachineModbus(unsigned char my_id_ = 0x11 , unsigned char friend_id_ = 0x01 , unsigned char port_num_ = 1)
    {
        my_id = my_id_;   //0x11 means slave ,  read Hands Free Link Manua.doc for detail
        friend_id = friend_id_;   // 0x01 means master
        port_num = port_num_;
        modbus_receive_state_=0;
        
    }
    inline unsigned char* getSerializedData(void)
    {
        return tx_buffer;
    }
    inline int getSerializedLength(void)
    {
        return tx_buffer_length;
    }
    inline int getSendValidData(void)
    {
        int temp=tx_buffer[4]<<8;
        temp=temp+tx_buffer[5];
        return temp;
    }
    
    unsigned char receiveStates(unsigned char rx_data);
    unsigned char receiveStates_R(unsigned char rx_data);
 

    void Request03(const HFMessageModbus* tx_message_);//批量读保持寄存器
    void Request04(const HFMessageModbus* tx_message_);//批量读输入寄存器
    void Request04ToPAD(const HFMessageModbus* tx_message_);
    void Request06(const HFMessageModbus* tx_message_);//单个写保持寄存器
    void Request16(const HFMessageModbus* tx_message_, unsigned short int len);//批量写保持寄存器
    void Request16ToPAD(const HFMessageModbus* tx_message_);//批量写保持寄存器
    void Request05(const HFMessageModbus* tx_message_);//继电器开
    void Request00(const HFMessageModbus* tx_message_); //继电器关

    void McMBCRC16(unsigned char *pDataIn, int iLenIn, unsigned short int  *pCRCOut);
    void HexToAscii(unsigned char * pHex, unsigned char * pAscii, int nLen);
    HFMessageModbus rx_message ,  tx_message;
    unsigned char my_id , friend_id;
    unsigned char tx_to_PAD_buffer[2048];
    unsigned short int tx_to_PAD_num;
    

private:
    RecstateModbus   receive_state_;
    float receive_message_count  , send_message_count;
    unsigned char tx_buffer[MODBUS_MESSAGE_BUFER_SIZE];
    unsigned char rx_buffer[MODBUS_MESSAGE_BUFER_SIZE];



    
    unsigned int receive_check_sum_;
    short int receive_message_length_;
    short int byte_count_;
    unsigned char port_num;
    unsigned tx_buffer_length;
    unsigned int modbus_receive_state_;
    unsigned short int CrcRecValue;
};

#endif // HF_LINK_STATE_MACHINE_H
