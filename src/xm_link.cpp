#include "xm_hw/xm_link.h"
#include <iomanip>


xm_link::xm_link(uint8_t my_id, uint8_t friend_id,RobotAbstract* my_robot)
{
    MyID=my_id; //masterID
    SlaveID=friend_id;  //slaveID
    receive_state_=WAITING_FF1;
    robot_ = my_robot;

    receive_package_count=0;
    receive_package_frequence=0;
    CheckSum=0;
    data_byte_count =0;
    rx_message_.sender_id=0;
    rx_message_.receiver_id=0;
    rx_message_.length=0;
    tx_message_.sender_id=0;
    tx_message_.receiver_id=0;
    tx_message_.length=0;
    //tx_buffer.resize(MESSAGE_BUFFER_SIZE);
    memset(receive_package_renew,0,sizeof(receive_package_renew));

}

//std::vector<uint8_t> xm_link::getSerialMessage()
//{
  //  return tx_buffer;
//}

unsigned char xm_link::byteAnalysisCall(const uint8_t rx_byte)
{
    unsigned char newPackage;
    unsigned char message_update;
    message_update=0;
    newPackage = receiveFiniteStates(rx_byte);

    if(newPackage == 1 )
    {
        message_update=packageAnalysis();
        receive_package_count++;
	return message_update;
	
    }
    return 0;
}

unsigned char xm_link::receiveFiniteStates(const uint8_t rx_data)
{

    switch (receive_state_)
    {
    case WAITING_FF1:
        if (rx_data == 0xff)
        {
            receive_state_ = WAITING_FF2;
            CheckSum =0;
            receive_message_length = 0;
            //byte_count_=0;
            CheckSum += rx_data;
        }
        break;

    case WAITING_FF2:
        if (rx_data == 0xff)
        {
            receive_state_ = SENDER_ID;
            CheckSum += rx_data;
        }
        else
            receive_state_ = WAITING_FF1;
        break;

    case SENDER_ID:
        rx_message_.sender_id = rx_data ;
        if (rx_message_.sender_id == SlaveID)  //id check
        {
            CheckSum += rx_data;
            receive_state_ = RECEIVER_ID;
        }
        else
        {
            std::cout<<"error , the sender_ID is not my Slave \n"<<std::endl;
            receive_state_ = WAITING_FF1;
        }
        break;

    case RECEIVER_ID:
        rx_message_.receiver_id = rx_data ;
        if (rx_message_.receiver_id == MyID)  //id check
        {
            CheckSum += rx_data;
            receive_state_ = RECEIVE_LEN_H;
        }
        else
        {
            std::cout<<"error , the reciver_ID is not my_ID \n"<<std::endl;
            receive_state_ = WAITING_FF1;
        }
        break;

    case RECEIVE_LEN_H:
        CheckSum += rx_data;
        receive_message_length |= rx_data<<8;
        receive_state_ = RECEIVE_LEN_L;
        break;

    case RECEIVE_LEN_L:
        CheckSum += rx_data;
        receive_message_length |= rx_data;
        rx_message_.length = receive_message_length - 1;//长度包括了校验位
        receive_state_ = RECEIVE_PACKAGE;
        break;

    case RECEIVE_PACKAGE:
        CheckSum += rx_data;
        rx_message_.data[data_byte_count++] = rx_data;
        if(data_byte_count >= (receive_message_length - 1))
        {	
	data_byte_count=0;
              receive_state_ = RECEIVE_CHECK;
              CheckSum=CheckSum%255;
        }
        break;
    case RECEIVE_CHECK:
        if(rx_data == (uint8_t)CheckSum)
        {
            CheckSum=0;
            receive_state_ = WAITING_FF1;
            //std::cout<<"receive a package \n"<<std::endl;
            return 1 ;
        }
        else
        {
            std::cout<<"check sum error !!!!!!!!!!!!!!!!!!!!!!!!!!\n"<<std::endl;
            std::cout<<std::hex<<"666"<<int(rx_message_.data[0])<<std::endl;
            receive_state_ = WAITING_FF1;
        }
        break;
    default:
        receive_state_ = WAITING_FF1;
    }
    return 0;
}

unsigned char xm_link::packageAnalysis()
{
 /*   
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
    */
    //std::cout<<"Package Analysis start  "<<std::endl;
    unsigned char analysis_state=0;
    Command command_state= (Command)rx_message_.data[0];
    //for(int i = 1 ;i < rx_message_.length ; i++) std::cout<<std::hex<<int(rx_message_.data[i]);
    //std::cout<<std::endl;
    switch (command_state)
       {
       case SET_ROBOT_SPEED:
            analysis_state=setCommandAnalysis(command_state,0,0);
            break;
       case SET_ARM_POSITION:
            analysis_state=setCommandAnalysis(command_state,0,0);
            break;
       case SET_HEAD_YAW_POSITION:
            analysis_state=setCommandAnalysis(command_state,0,0);
            break;
       case SET_HEAD_PITCH_POSITION:
            analysis_state=setCommandAnalysis(command_state,0,0);
            break;

       case SET_HAND_POSITION:
            analysis_state=setCommandAnalysis(command_state,0,0);

       case CLEAR_COORDINATE_DATA :
            analysis_state=setCommandAnalysis(command_state,0,0);
            break;


       case READ_ROBOT_SPEED :
            analysis_state=readCommandAnalysis(command_state , (unsigned char *)&robot_->measure_robot_velocity , sizeof(robot_->measure_robot_velocity));
            break;
       case READ_GLOBAL_COORDINATE :
            analysis_state=readCommandAnalysis(command_state , (unsigned char *)&robot_->robot_pose, sizeof(robot_->robot_pose));
            break;
       case READ_ARM_POSITION :
            analysis_state=readCommandAnalysis(command_state , (unsigned char *)&robot_->measure_arm_positions , sizeof(robot_->measure_arm_positions));
            break;
       case READ_HEAD_YAW_POSITION:
            analysis_state=readCommandAnalysis(command_state,(unsigned char *)&((robot_->expect_head_position).yaw),sizeof((robot_->expect_head_position).yaw));
	    //std::cout<<"DEBUG  WTH  "<<(robot_->expect_head_position).yaw<<std::endl;
            break;
       case READ_HEAD_PITCH_POSITION:
            analysis_state=readCommandAnalysis(command_state,(unsigned char *)&((robot_->expect_head_position).pitch),sizeof((robot_->expect_head_position).pitch));
            break;
       case READ_HAND_POSITION:
            analysis_state=readCommandAnalysis(command_state,(unsigned char *)&robot_->endEffortState,sizeof(robot_->endEffortState));
            break;

       default :
            analysis_state=0;
           break;

       }
       rx_message_.sender_id=0;    //clear flag 
       rx_message_.receiver_id=0;
       rx_message_.length=0;
       rx_message_.data[0]=0;
       return analysis_state;
}

unsigned char xm_link::readCommandAnalysis(const Command command ,  uint8_t* p , const unsigned short int len)
{

   // if((rx_message_.length-1) != len){
   //     std::cout<<"The length is not matching to struct \n"<<std::endl; // rx_mseeage_.data[0] = Command;
   //     return 0;
   // }
    memcpy(p,&rx_message_.data[1],len);
    for(int i=1; i<len+1 ; i++) std::cout<<std::hex<<int(rx_message_.data[i])<<" ";
    std::cout<<std::endl;
    receive_package_renew[(unsigned char)command] = 1 ;
    return 1;

}

unsigned char xm_link::setCommandAnalysis(const Command command,const uint8_t *p, const unsigned short int len)
{
    receive_package_renew[(unsigned char)command] = 1 ;
    std::cout<<"I received an ack \n"<<std::endl;
    return 1;
}

unsigned char xm_link::masterSendCommand(const Command command_state)
{

    //else  means master send a command to slave
    unsigned char analysis_state =1;
    switch (command_state)
    {
        //expect_arm_positions,measure_arm_positions

    case SET_ROBOT_SPEED :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (uint8_t *)&robot_->expect_robot_velocity , sizeof(robot_->expect_robot_velocity));
        break;

    case SET_ARM_POSITION:
        receive_package_renew[(unsigned char)command_state] = 0;
        sendStruct(command_state,(uint8_t *)&robot_->expect_arm_positions,sizeof(robot_->expect_arm_positions));
        break;  
    case SET_HEAD_YAW_POSITION:
        receive_package_renew[(unsigned char)command_state] = 0;
        sendStruct(command_state,(uint8_t *)&((robot_->expect_head_position).yaw),sizeof((robot_->expect_head_position).yaw));
        break;
    case SET_HEAD_PITCH_POSITION:
        receive_package_renew[(unsigned char)command_state] = 0;
        sendStruct(command_state,(uint8_t *)&((robot_->expect_head_position).pitch),sizeof((robot_->expect_head_position).pitch));
        break;
    case SET_HAND_POSITION:
        receive_package_renew[(unsigned char)command_state] =0;
        sendStruct(command_state,(uint8_t *)&robot_->endEffortCommand,sizeof(robot_->endEffortCommand));
        break;
    case CLEAR_COORDINATE_DATA:
        receive_package_renew[(unsigned char)command_state] =0;
        sendStruct(command_state,0,0);
        break;
        
    case READ_ROBOT_SPEED :
        receive_package_renew[(unsigned char)command_state] =0;
        sendStruct(command_state,0,0);

        break;
    case READ_GLOBAL_COORDINATE :
        receive_package_renew[(unsigned char)command_state] =0;
        sendStruct(command_state,0,0);
        break;
    case READ_ARM_POSITION :
        receive_package_renew[(unsigned char)command_state] =0;
        sendStruct(command_state,0,0);
        break;
    case READ_HEAD_YAW_POSITION:
        receive_package_renew[(unsigned char)command_state] =0;
        sendStruct(command_state,0,0);
        break;
    case READ_HEAD_PITCH_POSITION:
        receive_package_renew[(unsigned char)command_state] =0;
        sendStruct(command_state,0,0);
        break;
    default :
        analysis_state=0;
        break;
    }


    return analysis_state;
}


void xm_link::sendStruct(const Command command_state , unsigned char* p , const unsigned short int len)// length   of robot data 
{
        tx_message_.sender_id = MyID;
        tx_message_.receiver_id = SlaveID;
        tx_message_.length= len+ 1 + 1; //len = sizeof(data_type) 1 command  1 checkSum   	//FF FF ID ID len_H len_L command data[len] CheckSum
        tx_message_.data[0] =(uint8_t)command_state;
        unsigned int check_sum_=0;
         if(p!=NULL)
         {
               memcpy(&tx_message_.data[1] , p , len);
         } 
         tx_buffer[0]=0xff;
         check_sum_ += 0xff;
         tx_buffer[1]=0xff;
         check_sum_ += 0xff;
          tx_buffer[2]=tx_message_.sender_id;
          check_sum_ += tx_buffer[2];
          tx_buffer[3]=tx_message_.receiver_id;
          check_sum_ += tx_buffer[3];
          tx_buffer[4]=(uint8_t)( tx_message_.length >> 8); //LEN_H
          check_sum_ += tx_buffer[4];
          tx_buffer[5]=(uint8_t)tx_message_.length;   //LEN_L
          check_sum_ += tx_buffer[5];
          int tx_i;
          for( tx_i=0; tx_i < len+1  ; tx_i++)   // command  + len = len + 1  
          {
          	tx_buffer[6+tx_i]=tx_message_.data[tx_i];
          	check_sum_ += tx_buffer[6+tx_i];
          }
          check_sum_=check_sum_%255;
          tx_buffer[6+tx_i]= check_sum_;
          tx_buffer_length = 7+tx_i +1; //ff ff 01 01 len_H len_L command  data check_sum
          send_package_count++;
        }




