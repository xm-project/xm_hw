#include "xm_hw/xm_hw.h"
#include <iomanip>
#include <iostream>
namespace xm_hw {


xm_hw::xm_hw(std::string url,std::string config_addr)
{
    std::string transport_method = url.substr(0,url.find("://"));
    if(transport_method == "serial")
    {
        port_ = boost::make_shared<TransportSerial>(url);
        time_out_ = 500;
        Xm_Link = boost::make_shared<xm_link>(0x01,0x01,&robot_);
       // timer_.reset(new boost::asio::deadline_timer(*(port_->getIOinstace()),boost::posix_time::milliseconds(time_out_)));

    }

    else if (transport_method =="udp")
    {

    }
    else if (transport_method == "tcp")
    {

    }

    //process the config file
    file_.open(config_addr.c_str(), std::fstream::in);// 打开配置文件，配置相关的频率等参数
    if (file_.is_open())
    {
        for (int i = 0; i < LAST_COMMAND_FLAG; i++)
        {
            std::string temp;
            file_ >> temp >> xm_link_command_set[i] >> xm_link_command_freq[i];
            if((xm_link_command_freq[i] ==0)&&(xm_link_command_set[i] != 0))
            {
                xm_link_command_freq[i] = 1;
                std::cout<<"WARNING !!the freq of command :"<< i<<" was set to 1 defult!!!!!!!!!!!"<<std::endl;
            }
            std::cout<< temp <<"  "<<"Set:"<< xm_link_command_set[i] <<"   "<<"Freq: "<< xm_link_command_freq[i]<<std::endl;
        }
        file_.close();
        initialize_ok_ = port_->initialize_ok();
    } else
    {
        std::cerr << "config file can't be opened, check your system" <<std::endl;
        initialize_ok_ = false;
    }


}

bool xm_hw::updateCommand(const Command &command, int count)
{
    boost::asio::deadline_timer cicle_timer_(*(port_->getIOinstace()));
    cicle_timer_.expires_from_now(boost::posix_time::millisec(10));//time 10ms   100HZ
    if (xm_link_command_set[command] != 0)
    {
        int cnt = count % 100;
        if (cnt %  (100 / xm_link_command_freq[command]) == 0)
        {
            sendCommand(command);
        }
        else{
        	return false;
        }
      //      Buffer temp_read_buf_;  !!!!!!!!!!!!!!!!!!!!

    //Buffer data;
    ack_ready_ = false;
    while (!ack_ready_)
    {
        //data = port_->readBuffer();
        temp_buf  = port_->readBuffer();
        for (int i = 0; i < temp_buf.size(); i++)
        {
            if (Xm_Link->byteAnalysisCall(temp_buf[i]))
            {
                ack_ready_ = true;
            }
        }
        if (cicle_timer_.expires_from_now().is_negative())
        {
            std::cerr<<"Timeout continue skip this package"<<std::endl;
            return false;
        }
    }
    }
    return true;
}
}


