#include "xm_hw/xm_hw_ros.h"
namespace xm_hw {


xm_hw_ros::xm_hw_ros(ros::NodeHandle &nh, std::string url , std::string config_addr):
    xm_HW(url,config_addr),nh_(nh)
{
    nh_.setCallbackQueue(&queue_);
    base_mode_ = "3omni-wheel"; //you can also use "2diff-wheels"
    with_arm_ = false;
    controller_freq_ = 100;
    nh.getParam("/xm_hw/with_arm", with_arm_);
    nh.getParam("/xm_hw/freq",controller_freq_);
    
    position_x_ = position_y_ = pose_yaw_ = x_cmd_ = y_cmd_ = theta_cmd_=0.0;
    x_vel_ = y_vel_ = theta_vel_ =0.0;
    endEffort_state_ = 0.0;
    endEffort_command_ = 0.0;

    //base_state and base_cmd interfece .
    hardware_interface::BaseStateHandle base_state_handle("mobile_base",&position_x_,&position_y_,&pose_yaw_,&x_vel_ ,&y_vel_,&theta_vel_);
    base_state_interface_.registerHandle(base_state_handle);
    registerInterface(&base_state_interface_);

    hardware_interface::BaseVelocityHandle base_velocity_handle(base_state_handle,&x_cmd_,&y_cmd_,&theta_cmd_);
    base_velocity_interface_.registerHandle(base_velocity_handle);
    registerInterface(&base_velocity_interface_);

    // for arm joint
    if (with_arm_)  //use the ros_control interface to control the arm
    {
	arm_pos_.resize(5,0.0);
	arm_vel_.resize(5,0.0);
	arm_eff_.resize(5,0.0);
	arm_cmd_.resize(5,0.0);
	std::vector<std::string>joint_names;
	joint_names.push_back("joint_waist");
	joint_names.push_back("joint_big_arm");
	joint_names.push_back("joint_forearm");
	joint_names.push_back("joint_wrist_pitching");
	joint_names.push_back("joint_wrist_rotation");
            for (int i = 0;i < 5;i++)
            {
                hardware_interface::JointStateHandle arm_state_handle(joint_names[i], &arm_pos_[i], &arm_vel_[i], &arm_eff_[i]);
                joint_state_interface_.registerHandle(arm_state_handle);
                hardware_interface::JointHandle arm_handle(arm_state_handle , &arm_cmd_[i]);
                position_joint_interface_.registerHandle(arm_handle);
                ROS_WARN("registerHandle %d",i);
            }
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    //for head_servo
    hardware_interface::HeadServoStateHandle head_servo_state_handle("xm_head",&head_servo_yaw_pos_,&head_servo_pitch_pos_);
    head_servo_state_interface_.registerHandle(head_servo_state_handle);
    registerInterface(&head_servo_state_interface_);
    hardware_interface::HeadServoHandle head_servo_handle(head_servo_state_handle,&head_servo_yaw_cmd_,&head_servo_pitch_cmd_);
    head_servo_cmd_interface.registerHandle(head_servo_handle);
    registerInterface(&head_servo_cmd_interface);

    if(xm_HW.initialize_ok())
    {
        ROS_INFO("system initialized succeed, Fire In The Hole!!!!!!!! ");
    }
    else
    {
       ROS_ERROR("xm_HW initialized failed, please check the hardware");
    }

}


bool xm_hw_ros::serviceCallBack(xm_msgs::xm_Gripper::Request &req,xm_msgs::xm_Gripper::Response &res)
{
//1合上　０张开
    if(req.command==true)
    {
        endEffort_command_ = 1.0;
        res.result = true;
        res.message = "close";
        ROS_ERROR("DEBUG");
    }
    else{
        endEffort_command_  = 0.0;
        res.result = true;
        res.message = "open";
         ROS_ERROR("DEBUG");
    }
    return true;
}

void xm_hw_ros::mainloop()
{
    ros::CallbackQueue cm_callback_queue;
    ros::NodeHandle cm_nh_("xm_robot");
    cm_nh_.setCallbackQueue(&cm_callback_queue);
    controller_manager::ControllerManager cm(this,cm_nh_);

    ros::AsyncSpinner cm_spinner(1,&cm_callback_queue);
    ros::AsyncSpinner hw_spinner(1,&queue_); 

    server_ = cm_nh_.advertiseService("gripper_command", &xm_hw_ros::serviceCallBack,this);
    ros::Rate loop(controller_freq_);

    cm_spinner.start();
    hw_spinner.start();

    int count = 0;
    ros::Time currentTime = ros::Time::now();
    while(ros::ok())
    {
          xm_HW.updateCommand(SET_ROBOT_SPEED, count);
          xm_HW.updateCommand(READ_ROBOT_SPEED,count);
          xm_HW.updateCommand(READ_GLOBAL_COORDINATE,count);
          if(with_arm_)
        {
          xm_HW.updateCommand(READ_ARM_POSITION,count);
          xm_HW.updateCommand(SET_ARM_POSITION,count);
          xm_HW.updateCommand(SET_HAND_POSITION,count);
        }
          xm_HW.updateCommand(SET_HEAD_YAW_POSITION,count);
          //xm_HW.updateCommand(READ_HEAD_YAW_POSITION,count);
          xm_HW.updateCommand(SET_HEAD_PITCH_POSITION,count);
          //xm_HW.updateCommand(READ_HEAD_PITCH_POSITION,count);
        readBufferUpdate();
        cm.update(ros::Time::now(),ros::Duration(1.0/ controller_freq_));
        writeBufferUpdate();
        loop.sleep();
        count++;
        count=count%1000;
    }
    cm_spinner.stop();
    hw_spinner.stop();
}

}
