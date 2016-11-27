#ifndef XMROBOT_H
#define XMROBOT_H
//
//
//robot type
// 2: 2 wheel robot
//      balance car
//    differential car such as turtlebot
//    raider
// 3: 3 universal wheel robot
//
// some params use for the xm_robot

#define ROBOT_WHEEL_MODEL 3


#if ROBOT_WHEEL_MODEL == 2

#endif

#if ROBOT_WHEEL_MODEL == 3

#endif


#if ROBOT_WHEEL_MODEL == 4

#endif

struct RobotPose{
    float x;
    float y;
    float yaw;
};

struct RobotVelocity{
    float linear_x;
    float linear_y;
    float angular;
};

struct ArmJointPositions{
    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
};

struct ArmJointSpeed{
    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
};

struct ImuData{
    float roll;
    float pitch;
    float yaw;
};

struct HeadServo{
    float pitch;//俯仰角 上下
    float yaw;//偏航角  左右
};


struct SystemInfo{
    float   cpu_temperature;
    float   cpu_usage;
    float   battery_voltage;
};

struct RobotParameters{
    float robot_wheel_radius;
    float robot_body_radius;
    float speed_low_filter;
};

class RobotAbstract
{
    public:
        RobotVelocity expect_robot_velocity, measure_robot_velocity;
        RobotPose robot_pose;
        ArmJointPositions expect_arm_positions,measure_arm_positions;

        ArmJointSpeed expect_arm_joint_speed,measure_arm_joint_speed;

        HeadServo expect_head_position,measure_head_position;
        float endEffortState;
        float endEffortCommand;

    public:
        RobotAbstract()
        {
            expect_robot_velocity.linear_x=0;
            expect_robot_velocity.linear_y=0;
            expect_robot_velocity.angular=0;

            measure_robot_velocity.linear_x=0;
            measure_robot_velocity.linear_y=0;
            measure_robot_velocity.angular=0;

	    expect_arm_positions.joint1= 0;
	    expect_arm_positions.joint2= 0;
	    expect_arm_positions.joint3= 0;
	    expect_arm_positions.joint4= 0;
	    expect_arm_positions.joint5= 0;

	    measure_arm_positions.joint1= 0;
	    measure_arm_positions.joint2= 0;
	    measure_arm_positions.joint3= 0;
	    measure_arm_positions.joint4= 0;
	    measure_arm_positions.joint5= 0;

            measure_head_position.pitch = 0;
	    measure_head_position.yaw = 0;


            robot_pose.x = 0;
            robot_pose.y = 0;
            robot_pose.yaw = 0;
            
            endEffortState = 0; //1 means closed 
            endEffortCommand =0;
        }


};


#endif // XMROBOT_H
