#ifndef XM_HW_H
#define XM_HW_H
#include<iostream>
#include<fstream>
#include"xm_hw/transport_serial.h"
#include"xm_hw/xm_link.h"
#include"cstdlib"

namespace  xm_hw {


class xm_hw
{
public:
    xm_hw(std::string url,std::string config_addr);
    bool updateCommand(const Command &command,int count);
    void updateRobot();
    inline RobotAbstract* getRobotAbstract()
    {
        return &robot_;
    }
    inline boost::shared_ptr<boost::asio::io_service> getIOinstace()
    {
        return  port_->getIOinstace();
    }

    inline bool initialize_ok () const
    {
        return initialize_ok_;
    }

private:
    boost::shared_ptr<Transport> port_;
    boost::shared_ptr<xm_link> Xm_Link;
    //boost::shared_ptr<boost::asio::deadline_timer> timer_;

    //for read config file
    std::fstream file_;
    bool initialize_ok_;

    //for updating data
    int xm_link_command_set[LAST_COMMAND_FLAG];//用于决定是否发送相关命令
    int xm_link_command_freq[LAST_COMMAND_FLAG];//命令的频率
    int xm_link_command_set_current[LAST_COMMAND_FLAG];

    int time_out_;
    bool time_out_flag_;
    boost::mutex wait_mutex_;
    bool ack_ready_;
    //Buffer temp_read_buf_;
    //Buffer temp_write_buf_;
    Buffer temp_buf;
    RobotAbstract robot_;


    void timeoutHandle(const boost::system::error_code &ec);

    inline uint8_t checkUpdate(const Command command_state)//用于发送之后等待下位机的收到应答
    {
        if (xm_link_command_set_current[command_state] & Xm_Link->getReceiveRenewFlag(command_state))
        {
           return 1;
        }
        if(xm_link_command_set_current[command_state] ==0) return 1;
        return 0;

    }

    inline void sendCommand(const Command command_state) //
    {
        Xm_Link->masterSendCommand(command_state);
        //这里可能有问题
        Buffer data(Xm_Link->getSerializedData(), (Xm_Link->getSerializedLength()-1)+ Xm_Link->getSerializedData());
        //for(int i =0; i<data.size();i++) std::cout<<std::hex<<int(data[i])<<" ";
        //std::cout<<std::endl;
        port_->writeBuffer(data);

    }

};



}

#endif // XM_HW_H
