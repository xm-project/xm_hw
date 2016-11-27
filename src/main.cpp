#include "xm_hw/xm_hw_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw");
    ros::NodeHandle nh("xm_hw");
    xm_hw::xm_hw_ros xm(nh, "serial:///dev/ttyUSB0", "/home/dell/catkin_ws/src/xm_hw/config/config.txt");

    xm.mainloop();
    return 0;
}
