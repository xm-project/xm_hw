#ifndef XM_LINK_H

#define XM_LINK_H

#include<iostream>
#include"xm_hw/xmrobot.h"
#include<vector>
#include<inttypes.h>
#include<cstdlib>
#include<cstring>

static const unsigned short int MESSAGE_BUFFER_SIZE = 100;  //limite one message's size

struct Message{
    uint8_t sender_id;
    uint8_t receiver_id;
    uint8_t length;
    uint8_t data[MESSAGE_BUFFER_SIZE];
};

//communicate state
enum ReceiveState{
    WAITING_FF1,
    WAITING_FF2,
    SENDER_ID,
    RECEIVER_ID,
    RECEIVE_LEN_H,
    RECEIVE_LEN_L,
    RECEIVE_PACKAGE,
    RECEIVE_CHECK
};
/*
小端顺序 
0x01 发送速度 vx, vy, v_theta  回复 command = 0x01 数据 :无 
0x02 读取速度 vx,vy，theta(角度)            
0x03 读取里程 
0x04 发送机械臂命令  1 2 3 4 5 
0x05 读取机械臂位置
0x06 发送爪子命令
0x07 读取爪子
0x08 设置头部yaw  servo1 
0x09 设置头部pitch servo2  
0x0A 读取头部位置 yaw
0x0B 读取头部位置 pitch
0x0C 重置里程
*/
enum Command{
    EMPTY_CMD,//0x00
    SET_ROBOT_SPEED,//0x01
    READ_ROBOT_SPEED,//0x02
    READ_GLOBAL_COORDINATE,//0x03
    SET_ARM_POSITION,//0x04
    READ_ARM_POSITION,//0x05
    SET_HAND_POSITION,//0X06
    READ_HAND_POSITION,//0X07
    SET_HEAD_YAW_POSITION,//0X08 
    SET_HEAD_PITCH_POSITION,//0X09
    READ_HEAD_YAW_POSITION,//0X0A
    READ_HEAD_PITCH_POSITION,//0X0B
    CLEAR_COORDINATE_DATA,//0X0C
    LAST_COMMAND_FLAG};

class xm_link
{
public:
    xm_link(uint8_t my_id, uint8_t friend_id,RobotAbstract* my_robot);
    unsigned char masterSendCommand(const Command command_state);
    unsigned char byteAnalysisCall(const uint8_t rx_byte);
    //std::vector<uint8_t> getSerialMessage();
    inline uint8_t * getSerializedData(void) 
    {
        return tx_buffer;
    }
    inline int getSerializedLength(void)
    {
        return tx_buffer_length;
    }
    inline unsigned char getReceiveRenewFlag(const Command command_state) const
    {
        return receive_package_renew[command_state];
    }

private:
    unsigned char packageAnalysis(void);//for receive data to update the robot

    //when get the command,prepare a package to send
    unsigned char preparePackage(const Command command ,const uint8_t* p , const unsigned short int len);
    //receive state mechine
    unsigned char receiveFiniteStates(const uint8_t rx_data);
    //analysis the Send or Receive command
    //unsigned char commandAnalyis(const Command command , const uint8_t* p , const unsigned short int len);
    unsigned char readCommandAnalysis(const Command command ,  uint8_t* p , const unsigned short int len);
    unsigned char setCommandAnalysis(const Command command,const uint8_t* p, const unsigned short int len);
    void sendStruct(const Command command_state , unsigned char* p , const unsigned short int len);
private:
    uint8_t MyID; //masterID
    uint8_t SlaveID;  //slaveID
    ReceiveState receive_state_;
    RobotAbstract* robot_;

    int receive_package_count;
    int receive_package_frequence;
    int send_package_count;
    int data_byte_count;

    short int receive_message_length; // two bytes
    unsigned int CheckSum;
    unsigned char receive_package_renew[LAST_COMMAND_FLAG];

    Message rx_message_,tx_message_;
   // std::vector<uint8_t> tx_buffer;
    unsigned char tx_buffer[MESSAGE_BUFFER_SIZE + 20];
    unsigned tx_buffer_length;

};

#endif // XM_LINK_H
