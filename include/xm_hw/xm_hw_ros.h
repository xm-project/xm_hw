#ifndef XM_HW_ROS_H
#define XM_HW_ROS_H


#include <vector>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>


#include <xm_hw/base_cmd_interface.h>
#include <xm_hw/base_state_interface.h>
#include <xm_hw/head_servo_command_interface.h>
#include <xm_hw/head_servo_state_interface.h>

#include <xm_hw/transport.h>
#include <xm_hw/transport_serial.h>
#include <xm_hw/xm_link.h>
#include <xm_hw/xm_hw.h>
#include <xm_msgs/xm_Gripper.h>


namespace xm_hw {



class xm_hw_ros :public hardware_interface::RobotHW
{
public:
    xm_hw_ros(ros::NodeHandle &nh,std::string url,std::string config_addr);

    double inline getFreq() const
    {
        return controller_freq_;
    }
    void mainloop();
    bool serviceCallBack(xm_msgs::xm_Gripper::Request &req,xm_msgs::xm_Gripper::Response &res);

private:
    xm_hw xm_HW;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;
    ros::ServiceServer  server_;

    std::string base_mode_;
    bool with_arm_;
    double controller_freq_;


    std::vector<double> arm_pos_ , arm_vel_, arm_eff_ , arm_cmd_;  //to control the arm of five dof
    double endEffort_command_,endEffort_state_; //control to pick or place the Object

    double position_x_,position_y_,pose_yaw_;
    double x_vel_ , y_vel_ ,theta_vel_;

    double x_cmd_,y_cmd_,theta_cmd_; //base_command


    double head_servo_yaw_pos_ ,head_servo_pitch_pos_;
    double head_servo_yaw_cmd_ ,head_servo_pitch_cmd_;


    hardware_interface::JointStateInterface joint_state_interface_;     //states of arm joints
    hardware_interface::PositionJointInterface position_joint_interface_;  //to control the arm


    hardware_interface::HeadServoStateInterface head_servo_state_interface_;
    hardware_interface::HeadServoInterface head_servo_cmd_interface ;

    hardware_interface::BaseStateInterface base_state_interface_;
    hardware_interface::BaseVelocityInterface base_velocity_interface_;


    inline void writeBufferUpdate()
    {
        /*
        xm_HW.getRobotAbstract()->expect_motor_speed.motor1 = wheel_cmd_[0];
        xm_HW.getRobotAbstract()->expect_motor_speed.motor2 = wheel_cmd_[1];
        xm_HW.getRobotAbstract()->expect_motor_speed.motor3 = wheel_cmd_[2];
        */
        xm_HW.getRobotAbstract()->expect_robot_velocity.linear_x = x_cmd_;
        xm_HW.getRobotAbstract()->expect_robot_velocity.linear_y = y_cmd_;
        xm_HW.getRobotAbstract()->expect_robot_velocity.angular = theta_cmd_;
        xm_HW.getRobotAbstract()->expect_head_position.pitch = head_servo_pitch_cmd_;
        xm_HW.getRobotAbstract()->expect_head_position.yaw = head_servo_yaw_cmd_;
        if(with_arm_)
        {
            xm_HW.getRobotAbstract()->expect_arm_positions.joint1 = arm_cmd_[0];
            xm_HW.getRobotAbstract()->expect_arm_positions.joint2 = arm_cmd_[1];
            xm_HW.getRobotAbstract()->expect_arm_positions.joint3 = arm_cmd_[2];
            xm_HW.getRobotAbstract()->expect_arm_positions.joint4 = arm_cmd_[3];
            xm_HW.getRobotAbstract()->expect_arm_positions.joint5 = arm_cmd_[4];
            xm_HW.getRobotAbstract()->endEffortCommand = endEffort_command_;
        }
    }

    inline void readBufferUpdate()
    {
        position_x_ = xm_HW.getRobotAbstract()->robot_pose.x;
        position_y_ = xm_HW.getRobotAbstract()->robot_pose.y;
        pose_yaw_ = xm_HW.getRobotAbstract()->robot_pose.yaw;
        
        x_vel_ =xm_HW.getRobotAbstract()->measure_robot_velocity.linear_x;
        y_vel_ =xm_HW.getRobotAbstract()->measure_robot_velocity.linear_y;
        theta_vel_ = xm_HW.getRobotAbstract()->measure_robot_velocity.angular;
        //ROS_INFO("head_servo yaw: %f , pitch: %f ",xm_HW.getRobotAbstract()->measure_head_position.yaw,xm_HW.getRobotAbstract()->measure_head_position.pitch);
/*
        head_servo_yaw_pos_ = xm_HW.getRobotAbstract()->measure_head_position.yaw;
        head_servo_pitch_pos_ = xm_HW.getRobotAbstract()->measure_head_position.pitch;
*/
	head_servo_pitch_pos_ = head_servo_pitch_cmd_;
	head_servo_yaw_pos_ = head_servo_yaw_cmd_;
	//F4 发送舵机数据有问题,暂时用这种BUG方法

        if(with_arm_)
        {
            arm_pos_[0] = xm_HW.getRobotAbstract()->measure_arm_positions.joint1;
            arm_pos_[1] = xm_HW.getRobotAbstract()->measure_arm_positions.joint2;
            arm_pos_[2] = xm_HW.getRobotAbstract()->measure_arm_positions.joint3;
            arm_pos_[3] = xm_HW.getRobotAbstract()->measure_arm_positions.joint4;
            arm_pos_[4] = xm_HW.getRobotAbstract()->measure_arm_positions.joint5;
            endEffort_state_ = endEffort_command_;

        }
    }







};

}
#endif // XM_HW_ROS_H
